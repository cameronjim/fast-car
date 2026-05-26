"""
SAC training node (simulator).

Drives the car, collects transitions, and trains actor/critics online.
Episode boundaries come from the safety node's /kys topic. When /kys
latches, the episode ends and the car is reset to the starting pose.
Training resumes automatically when the safety node releases /kys.

Launch:
    ros2 launch learned_control sac_train_launch.py
"""
from __future__ import annotations

import math
import os
import csv
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

from learned_control.sac.model import SACActorNet, SACCriticNet
from learned_control.sac.train import SACTrainer
from learned_control.sac.reward import compute_reward

LIDAR_STEP = 6
MAX_RANGE = 10.0


class SACTrainNode(Node):
    """
    This class defines the SAC training node.

    Drives the car, collects transitions into a replay buffer, and runs
    online SAC gradient updates. Episode boundaries are signalled by the
    safety node via /kys.
    """

    def __init__(self) -> None:
        """
        Initializes the SAC training node.

        Args:
            None

        Returns:
            None
        """
        super().__init__("sac_train_node")

        # Parameters
        self.declare_parameter("bc_weights_path", "")
        self.declare_parameter("scalers_path", "")
        self.declare_parameter("initial_checkpoint_path", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("log_path", "")
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("deterministic", False)
        self.declare_parameter("resume_training", False)
        self.declare_parameter("lr_actor", 1e-4)
        self.declare_parameter("lr_critic", 3e-4)
        self.declare_parameter("gamma", 0.99)
        self.declare_parameter("tau", 0.005)
        self.declare_parameter("buffer_size", 100000)
        self.declare_parameter("batch_size", 256)
        self.declare_parameter("update_every", 10)
        self.declare_parameter("warmup_steps", 2000)
        self.declare_parameter("learning_starts", 3000)
        self.declare_parameter("actor_learning_starts", 10000)
        self.declare_parameter("bc_reg_weight", 2.0)
        self.declare_parameter("bc_reg_decay_steps", 50000)
        self.declare_parameter("save_every", 5000)
        self.declare_parameter("reset_x", 0.0)
        self.declare_parameter("reset_y", 0.0)
        self.declare_parameter("reset_yaw", 0.0)
        self.declare_parameter("odom_topic", "/ego_racecar/odom")

        # Read the parameters from the launch file
        bc_weights_path = self._str("bc_weights_path")
        scalers_path = self._str("scalers_path")
        initial_checkpoint_path = self._str("initial_checkpoint_path")
        self.checkpoint_path = self._str("checkpoint_path")
        self.log_path = self._str("log_path")
        self.max_speed = self._dbl("max_speed")
        self.min_speed = self._dbl("min_speed")
        self.deterministic = self._bool("deterministic")
        resume_training = self._bool("resume_training")
        lr_actor = self._dbl("lr_actor")
        lr_critic = self._dbl("lr_critic")
        gamma = self._dbl("gamma")
        tau = self._dbl("tau")
        buffer_size = self._int("buffer_size")
        batch_size = self._int("batch_size")
        self.update_every = self._int("update_every")
        self.warmup_steps = self._int("warmup_steps")
        self.learning_starts = self._int("learning_starts")
        self.actor_learning_starts = self._int("actor_learning_starts")
        self.bc_reg_weight = self._dbl("bc_reg_weight")
        self.bc_reg_decay_steps = self._int("bc_reg_decay_steps")
        self.save_every = self._int("save_every")
        self.reset_x = self._dbl("reset_x")
        self.reset_y = self._dbl("reset_y")
        self.reset_yaw = self._dbl("reset_yaw")

        # Load the scalers from the .npz file
        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)
        self.num_lidar = len(self.lidar_scale)
        self.get_logger().info(f"LiDAR features: {self.num_lidar}")

        # Build and load the networks (actor and 2 critics)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        actor = SACActorNet(self.num_lidar)
        critic1 = SACCriticNet(self.num_lidar)
        critic2 = SACCriticNet(self.num_lidar)
        self.trainer = SACTrainer(
            actor, critic1, critic2,
            state_dim=self.num_lidar, lr_actor=lr_actor, lr_critic=lr_critic,
            lr_alpha=lr_critic, gamma=gamma, tau=tau,
            buffer_size=buffer_size, batch_size=batch_size, device=device,
        )

        self.has_initial_policy = False

        # Load the initial checkpoint if it exists and resume training is False
        if initial_checkpoint_path and os.path.isfile(initial_checkpoint_path):
            self.get_logger().info(
                f"Initialising from selected checkpoint: {initial_checkpoint_path}"
            )
            self.trainer.load(initial_checkpoint_path)
            self.has_initial_policy = True
        elif resume_training and os.path.isfile(self.checkpoint_path):
            self.get_logger().info(f"Resuming from checkpoint: {self.checkpoint_path}")
            self.trainer.load(self.checkpoint_path)
            self.has_initial_policy = True
        elif bc_weights_path and os.path.isfile(bc_weights_path):
            self.get_logger().info(f"Initialising actor from BC: {bc_weights_path}")
            bc_actor = SACActorNet.from_bc(bc_weights_path, self.num_lidar, device=device)
            self.trainer.actor.load_state_dict(bc_actor.state_dict())
            self.has_initial_policy = True
            if os.path.isfile(self.checkpoint_path):
                self.get_logger().info(
                    "Ignoring existing training checkpoint; resume_training is false"
                )
        else:
            self.get_logger().warn("No checkpoint or BC weights -- random init")

        # Log the status of the training node
        self.get_logger().info(
            f"SAC TRAIN ready | deterministic={self.deterministic} device={device}"
        )
        # Set the reference actor if the initial policy exists
        if self.has_initial_policy:
            self.trainer.set_reference_actor(self.trainer.actor)

        # State tracking
        self.prev_state = None
        self.prev_action = None
        self.prev_raw_lidar = None
        self.prev_steering = 0.0
        self.prev_prev_steering = 0.0
        self.prev_speed_cmd = 0.0
        self.current_speed = 0.0
        self.step_count = 0
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.episode_count = 0
        self.best_episode_steps = 0
        self.stopped = False

        self._init_log()

        # ROS2 subscribers and publishers
        odom_topic = self._str("odom_topic")
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.kys_sub = self.create_subscription(
            Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive_raw", 10)
        self.reset_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

    # Helper functions to get the parameters from the launch file
    def _str(self, n) -> str:
        """
        Helper function to get the string value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The string value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().string_value

    def _dbl(self, n) -> float:
        """
        Helper function to get the double value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The double value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().double_value

    def _int(self, n) -> int:
        """
        Helper function to get the integer value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The integer value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().integer_value

    def _bool(self, n) -> bool:
        """
        Helper function to get the boolean value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The boolean value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().bool_value

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Callback function for the LaserScan topic.

        Preprocesses the LiDAR data, stores the previous transition with its
        reward, selects an action from the policy, publishes a drive command,
        and runs a gradient update step every update_every steps.

        Args:
            msg: The LaserScan message.

        Returns:
            None
        """
        if self.stopped:
            self._publish_stop()
            return

        raw_ranges = np.array(msg.ranges, dtype=np.float32)

        # Preprocess the LiDAR data to the range [0, 1]
        ds = raw_ranges[::LIDAR_STEP]
        ds = np.where(np.isfinite(ds), ds, MAX_RANGE)
        ds = np.clip(ds, 0.0, MAX_RANGE)
        if len(ds) > self.num_lidar:
            ds = ds[:self.num_lidar]
        elif len(ds) < self.num_lidar:
            ds = np.pad(ds, (0, self.num_lidar - len(ds)),
                        constant_values=MAX_RANGE)
        raw_lidar = ds.copy()
        state = ds * self.lidar_scale + self.lidar_min

        # Store the previous transition with its reward
        if self.prev_state is not None:
            reward = compute_reward(
                self.prev_raw_lidar, self.current_speed,
                self.prev_steering, done=False,
                prev_steering=self.prev_prev_steering,
            )
            self.trainer.store(
                self.prev_state, self.prev_action, reward, state, False)
            self.episode_reward += reward
            self.episode_steps += 1

        # Select an action from the policy
        if self.step_count < self.warmup_steps:
            if self.has_initial_policy:
                state_t = torch.from_numpy(state.reshape(1, -1)).to(
                    self.trainer.device)
                action = (self.trainer.actor.get_action(state_t, deterministic=True)
                          .cpu().numpy()[0])
            else:
                action = np.random.uniform(0.0, 1.0, size=2).astype(np.float32)
        else:
            state_t = torch.from_numpy(state.reshape(1, -1)).to(
                self.trainer.device)
            action = (self.trainer.actor.get_action(state_t, self.deterministic)
                      .cpu().numpy()[0])

        # Denormalize the predicted steering angle and speed to the original range
        steering = float((action[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((action[1] - self.action_min[1]) / self.action_scale[1])
        steering, speed = self._postprocess_action(steering, speed, raw_lidar)

        # Publish the steering angle and speed
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        # Bookkeeping
        self.prev_state = state
        self.prev_action = action
        self.prev_raw_lidar = raw_lidar
        self.prev_prev_steering = self.prev_steering
        self.prev_steering = steering
        self.step_count += 1

        # Run a gradient update step every update_every steps
        if self.step_count >= self.learning_starts and self.step_count % self.update_every == 0:
            # Set the BC regularization weight to 0.0 if the BC regularization decay steps is 0
            bc_weight = 0.0
            # Set the BC regularization weight to the BC regularization weight * the maximum of 0.0 and the difference between the current step count and the BC regularization decay steps
            if self.bc_reg_decay_steps > 0:
                bc_weight = self.bc_reg_weight * max(
                    0.0,
                    1.0 - self.step_count / float(self.bc_reg_decay_steps),
                )
            # Run a gradient update step on the actor and critics
            metrics = self.trainer.update(
                update_actor=self.step_count >= self.actor_learning_starts,
                bc_reg_weight=bc_weight,
            )
            # Log the metrics if the step count is a multiple of 200
            if metrics and self.step_count % 200 == 0:
                self.get_logger().info(
                    f"[step {self.step_count}] "
                    f"c1={metrics['critic1_loss']:.4f} "
                    f"c2={metrics['critic2_loss']:.4f} "
                    f"actor={metrics['actor_loss']:.4f} "
                    f"alpha={metrics['alpha']:.4f} "
                    f"bc={metrics['bc_loss']:.4f}")

        # Save a checkpoint every save_every steps
        if self.step_count % self.save_every == 0:
            self.trainer.save(self.checkpoint_path)
            self.get_logger().info(
                f"Checkpoint saved (step {self.step_count}, "
                f"buffer {len(self.trainer.buffer)})")

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for the Odometry topic.

        Args:
            msg: The Odometry message.

        Returns:
            None
        """
        self.current_speed = abs(msg.twist.twist.linear.x)

    def kys_callback(self, msg: Bool) -> None:
        """
        Callback function for the KYS topic.

        Ends the episode and resets the car when the safety node latches,
        and resumes driving when it releases.

        Args:
            msg: The Bool message.

        Returns:
            None
        """
        if msg.data and not self.stopped:
            self.stopped = True
            self._end_episode()
            self._reset_car()
        elif not msg.data and self.stopped:
            self.stopped = False

    def _publish_stop(self) -> None:
        """
        Publish a stop message.

        Args:
            None

        Returns:
            None
        """
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _postprocess_action(self, steering, speed, raw_lidar) -> tuple[float, float]:
        """
        Clamp and validate the predicted steering angle and speed.

        Args:
            steering: The predicted steering angle in radians.
            speed: The predicted speed in m/s.
            raw_lidar: The raw LiDAR distances in meters, shape (num_rays,).

        Returns:
            A tuple of (steering, speed) clamped to valid ranges.
        """
        if not np.isfinite(steering):
            steering = 0.0
        if not np.isfinite(speed):
            speed = self.min_speed

        # Never allow reverse commands from the learned policy
        speed = max(0.0, min(speed, self.max_speed))
        if 0.0 < speed < self.min_speed:
            speed = self.min_speed

        return steering, speed

    def _end_episode(self) -> None:
        """
        End the current episode, log results, and save a checkpoint.

        Stores the terminal transition with done=True, increments the episode
        counter, saves a new best checkpoint if this episode had the most steps,
        and resets episode state.

        Args:
            None

        Returns:
            None
        """
        # Store the terminal transition with done=True
        if self.prev_state is not None:
            reward = compute_reward(
                self.prev_raw_lidar, 0.0, self.prev_steering, done=True,
                prev_steering=self.prev_prev_steering)
            self.trainer.store(
                self.prev_state, self.prev_action, reward,
                self.prev_state, True)
            self.episode_reward += reward
            self.episode_steps += 1

        # Increment the episode counter
        self.episode_count += 1
        # Log the episode results
        self.get_logger().info(
            f"Episode {self.episode_count} | "
            f"reward={self.episode_reward:.2f} "
            f"steps={self.episode_steps} "
            f"total={self.step_count} "
            f"buffer={len(self.trainer.buffer)}")

        # Save the best checkpoint if the current episode has the most steps
        if self.episode_steps > self.best_episode_steps:
            self.best_episode_steps = self.episode_steps
            best_path = self.checkpoint_path.replace('.pth', '_best.pth')
            self.trainer.save(best_path)
            self.get_logger().info(
                f"NEW BEST (steps={self.episode_steps}, "
                f"reward={self.episode_reward:.2f})")

        # Log the episode results
        self._log_episode()
        self.trainer.save(self.checkpoint_path)

        # Reset the episode state
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.prev_state = None
        self.prev_action = None
        self.prev_speed_cmd = 0.0

    def _reset_car(self) -> None:
        """
        Reset the car to the starting pose by publishing to /initialpose.

        Args:
            None

        Returns:
            None
        """
        self._publish_stop()

        # Publish the starting pose
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.reset_x
        pose.pose.pose.position.y = self.reset_y
        pose.pose.pose.orientation.z = math.sin(self.reset_yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(self.reset_yaw / 2.0)
        self.reset_pub.publish(pose)

    def _init_log(self) -> None:
        """
        Initialize the training log CSV file with a header row.

        Args:
            None

        Returns:
            None
        """
        os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
        # Create the training log CSV file if it doesn't exist
        if not os.path.isfile(self.log_path):
            with open(self.log_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "episode", "reward", "steps", "total_steps", "buffer_size"])

    def _log_episode(self) -> None:
        """
        Append the current episode results to the training log CSV.

        Args:
            None

        Returns:
            None
        """
        # Append the current episode results to the training log CSV
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([
                self.episode_count,
                round(self.episode_reward, 4),
                self.episode_steps,
                self.step_count,
                len(self.trainer.buffer)])


def main(args=None) -> None:
    """
    Main function to initialize the ROS2 node.

    Args:
        args: The arguments.

    Returns:
        None
    """
    rclpy.init(args=args)
    node = SACTrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down — saving checkpoint")
        node.trainer.save(node.checkpoint_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
