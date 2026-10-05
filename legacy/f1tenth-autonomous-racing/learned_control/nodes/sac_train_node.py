"""ros node training sac online in the simulator, one env step per incoming scan."""
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
from learned_control.preprocessing.scan import downsample_scan, normalize_scan

METRICS_LOG_EVERY = 200


class SACTrainNode(Node):
    """collects transitions into a replay buffer and runs sac updates; /kys ends the episode."""

    def __init__(self) -> None:
        super().__init__("sac_train_node")

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

        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)
        # the exported scaler length is what fixes the policy input width
        self.num_lidar = len(self.lidar_scale)
        self.get_logger().info(f"lidar features: {self.num_lidar}")

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

        # built up front: it is both a possible actor init and the regularization reference
        bc_actor = None
        if bc_weights_path and os.path.isfile(bc_weights_path):
            bc_actor = SACActorNet.from_bc(bc_weights_path, self.num_lidar, device=device)

        if initial_checkpoint_path and os.path.isfile(initial_checkpoint_path):
            self.get_logger().info(
                f"initialising from selected checkpoint: {initial_checkpoint_path}"
            )
            self.trainer.load(initial_checkpoint_path)
            self.has_initial_policy = True
        elif resume_training and os.path.isfile(self.checkpoint_path):
            self.get_logger().info(f"resuming from checkpoint: {self.checkpoint_path}")
            self.trainer.load(self.checkpoint_path)
            self.has_initial_policy = True
        elif bc_actor is not None:
            self.get_logger().info(f"initialising actor from bc: {bc_weights_path}")
            self.trainer.actor.load_state_dict(bc_actor.state_dict())
            self.has_initial_policy = True
            if os.path.isfile(self.checkpoint_path):
                self.get_logger().info(
                    "ignoring existing training checkpoint, resume_training is false"
                )
        else:
            self.get_logger().warn("no checkpoint or bc weights, random init")

        self.get_logger().info(
            f"sac train ready | deterministic={self.deterministic} device={device}"
        )
        # the reference must be the frozen bc policy; the loaded sac actor would
        # regularize the policy towards its own starting weights
        if bc_actor is not None:
            self.trainer.set_reference_actor(bc_actor)
            self.get_logger().info(
                f"bc regularization reference: bc weights ({bc_weights_path})"
            )
        elif self.has_initial_policy:
            self.trainer.set_reference_actor(self.trainer.actor)
            self.get_logger().warn(
                "no bc weights available, regularizing towards the loaded "
                "checkpoint actor instead"
            )

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

    def _str(self, name) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _dbl(self, name) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int(self, name) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool(self, name) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def scan_callback(self, msg: LaserScan) -> None:
        """one training step: store the last transition, act, publish, and maybe update."""
        if self.stopped:
            self._publish_stop()
            return

        raw_lidar = downsample_scan(msg.ranges, self.num_lidar)
        state = normalize_scan(raw_lidar, self.lidar_scale, self.lidar_min)

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

        steering = float((action[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((action[1] - self.action_min[1]) / self.action_scale[1])
        steering, speed = self._postprocess_action(steering, speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        # store what was actually published: _postprocess_action clamps the command,
        # so the raw policy output would train the critics on actions never executed
        executed_action = np.array(
            [steering * self.action_scale[0] + self.action_min[0],
             speed * self.action_scale[1] + self.action_min[1]],
            dtype=np.float32,
        )
        executed_action = np.clip(executed_action, 0.0, 1.0)

        self.prev_state = state
        self.prev_action = executed_action
        self.prev_raw_lidar = raw_lidar
        self.prev_prev_steering = self.prev_steering
        self.prev_steering = steering
        self.step_count += 1

        if self.step_count >= self.learning_starts and self.step_count % self.update_every == 0:
            # bc regularization decays linearly to zero over bc_reg_decay_steps
            bc_weight = 0.0
            if self.bc_reg_decay_steps > 0:
                bc_weight = self.bc_reg_weight * max(
                    0.0,
                    1.0 - self.step_count / float(self.bc_reg_decay_steps),
                )
            metrics = self.trainer.update(
                update_actor=self.step_count >= self.actor_learning_starts,
                bc_reg_weight=bc_weight,
            )
            if metrics and self.step_count % METRICS_LOG_EVERY == 0:
                self.get_logger().info(
                    f"[step {self.step_count}] "
                    f"c1={metrics['critic1_loss']:.4f} "
                    f"c2={metrics['critic2_loss']:.4f} "
                    f"actor={metrics['actor_loss']:.4f} "
                    f"alpha={metrics['alpha']:.4f} "
                    f"bc={metrics['bc_loss']:.4f}")

        if self.step_count % self.save_every == 0:
            self.trainer.save(self.checkpoint_path)
            self.get_logger().info(
                f"checkpoint saved at step {self.step_count}, "
                f"buffer {len(self.trainer.buffer)}")

    def odom_callback(self, msg: Odometry) -> None:
        self.current_speed = abs(msg.twist.twist.linear.x)

    def kys_callback(self, msg: Bool) -> None:
        """end the episode and reset the car on a latch, resume on a release."""
        if msg.data and not self.stopped:
            self.stopped = True
            self._end_episode()
            self._reset_car()
        elif not msg.data and self.stopped:
            self.stopped = False

    def _publish_stop(self) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _postprocess_action(self, steering, speed) -> tuple[float, float]:
        """clamp the policy output at the boundary it crosses into ros."""
        if not np.isfinite(steering):
            steering = 0.0
        if not np.isfinite(speed):
            speed = self.min_speed

        # never allow reverse commands from the learned policy
        speed = max(0.0, min(speed, self.max_speed))
        if 0.0 < speed < self.min_speed:
            speed = self.min_speed

        return steering, speed

    def _end_episode(self) -> None:
        """store the terminal transition, log the episode, and checkpoint."""
        if self.prev_state is not None:
            reward = compute_reward(
                self.prev_raw_lidar, 0.0, self.prev_steering, done=True,
                prev_steering=self.prev_prev_steering)
            self.trainer.store(
                self.prev_state, self.prev_action, reward,
                self.prev_state, True)
            self.episode_reward += reward
            self.episode_steps += 1

        self.episode_count += 1
        self.get_logger().info(
            f"episode {self.episode_count} | "
            f"reward={self.episode_reward:.2f} "
            f"steps={self.episode_steps} "
            f"total={self.step_count} "
            f"buffer={len(self.trainer.buffer)}")

        # longest episode so far, kept separately from the rolling checkpoint
        if self.episode_steps > self.best_episode_steps:
            self.best_episode_steps = self.episode_steps
            best_path = self.checkpoint_path.replace('.pth', '_best.pth')
            self.trainer.save(best_path)
            self.get_logger().info(
                f"new best, steps={self.episode_steps} "
                f"reward={self.episode_reward:.2f}")

        self._log_episode()
        self.trainer.save(self.checkpoint_path)

        self.episode_reward = 0.0
        self.episode_steps = 0
        self.prev_state = None
        self.prev_action = None
        self.prev_speed_cmd = 0.0

    def _reset_car(self) -> None:
        """put the car back on the starting pose through /initialpose."""
        self._publish_stop()

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.reset_x
        pose.pose.pose.position.y = self.reset_y
        pose.pose.pose.orientation.z = math.sin(self.reset_yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(self.reset_yaw / 2.0)
        self.reset_pub.publish(pose)

    def _init_log(self) -> None:
        """create the training log csv with a header row if it does not exist yet."""
        os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
        if not os.path.isfile(self.log_path):
            with open(self.log_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "episode", "reward", "steps", "total_steps", "buffer_size"])

    def _log_episode(self) -> None:
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([
                self.episode_count,
                round(self.episode_reward, 4),
                self.episode_steps,
                self.step_count,
                len(self.trainer.buffer)])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SACTrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down, saving checkpoint")
        node.trainer.save(node.checkpoint_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
