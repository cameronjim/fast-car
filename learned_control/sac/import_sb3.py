"""
Convert a stable-baselines3 SAC model to a custom SAC checkpoint.

Usage (run on the Linux VM where SB3 and the model exist):
    python import_sb3.py --sb3-path ~/rl_models/sac_final.zip \
                          --out sac/sac_checkpoint.pth

This maps weights from SB3's MlpPolicy (256, 256 hidden) to our
custom SACActorNet/SACCriticNet with matching hidden sizes.
"""

import argparse
import torch
import torch.nn as nn

# Handle both ROS2 package imports and standalone execution
try:
    from learned_control.sac.model import SACActorNet, SACCriticNet
except ImportError:
    from model import SACActorNet, SACCriticNet

try:
    from learned_control.sac.train import SACTrainer
except ImportError:
    from train import SACTrainer


def convert(sb3_path: str, out_path: str, num_lidar: int = 181) -> None:
    """
    Convert a stable-baselines3 SAC checkpoint to our custom format.

    Args:
        sb3_path: Path to the SB3 .zip model file.
        out_path: Output path for the converted .pth checkpoint.
        num_lidar: Number of LiDAR rays (input features).

    Returns:
        None
    """
    # Import the SB3 SAC model
    from stable_baselines3 import SAC as SB3_SAC

    # Print the loading message
    print(f"Loading SB3 model from {sb3_path} ...")
    sb3_model = SB3_SAC.load(sb3_path, device="cpu")
    sd = sb3_model.policy.state_dict()

    # Print keys for debugging
    print("SB3 state dict keys:")
    for k, v in sd.items():
        print(f"  {k:50s} {tuple(v.shape)}")

    # Initialize the actor network
    # SB3 actor: latent_pi.0 (in→256), latent_pi.2 (256→256), mu (256→2), log_std (2,)
    actor = SACActorNet(num_lidar, action_dim=2, hidden1=256, hidden2=256)
    # Copy the weights to the actor network
    actor.fc1.weight.data.copy_(sd["actor.latent_pi.0.weight"])
    actor.fc1.bias.data.copy_(sd["actor.latent_pi.0.bias"])
    actor.fc2.weight.data.copy_(sd["actor.latent_pi.2.weight"])
    actor.fc2.bias.data.copy_(sd["actor.latent_pi.2.bias"])
    actor.mean_head.weight.data.copy_(sd["actor.mu.weight"])
    actor.mean_head.bias.data.copy_(sd["actor.mu.bias"])
    # Initialize the log standard deviation head
    nn.init.constant_(actor.log_std_head.weight, 0.0)
    actor.log_std_head.bias.data.copy_(sd["actor.log_std"])
    print("Actor weights copied.")

    # Initialize the critic networks
    # SB3 critic: qf0.{0,2,4} and qf1.{0,2,4}
    critic1 = SACCriticNet(num_lidar, action_dim=2, hidden1=256, hidden2=256)
    critic1.net[0].weight.data.copy_(sd["critic.qf0.0.weight"])
    critic1.net[0].bias.data.copy_(sd["critic.qf0.0.bias"])
    critic1.net[2].weight.data.copy_(sd["critic.qf0.2.weight"])
    critic1.net[2].bias.data.copy_(sd["critic.qf0.2.bias"])
    critic1.net[4].weight.data.copy_(sd["critic.qf0.4.weight"])
    critic1.net[4].bias.data.copy_(sd["critic.qf0.4.bias"])

    critic2 = SACCriticNet(num_lidar, action_dim=2, hidden1=256, hidden2=256)
    critic2.net[0].weight.data.copy_(sd["critic.qf1.0.weight"])
    critic2.net[0].bias.data.copy_(sd["critic.qf1.0.bias"])
    critic2.net[2].weight.data.copy_(sd["critic.qf1.2.weight"])
    critic2.net[2].bias.data.copy_(sd["critic.qf1.2.bias"])
    critic2.net[4].weight.data.copy_(sd["critic.qf1.4.weight"])
    critic2.net[4].bias.data.copy_(sd["critic.qf1.4.bias"])
    print("Critic weights copied.")

    # Initialize the trainer
    trainer = SACTrainer(
        actor, critic1, critic2,
        state_dim=num_lidar, action_dim=2,
        device="cpu",
    )

    # Copy target critics from SB3
    # Copy the weights to the target critic networks
    trainer.target_critic1.net[0].weight.data.copy_(sd["critic_target.qf0.0.weight"])
    trainer.target_critic1.net[0].bias.data.copy_(sd["critic_target.qf0.0.bias"])
    trainer.target_critic1.net[2].weight.data.copy_(sd["critic_target.qf0.2.weight"])
    trainer.target_critic1.net[2].bias.data.copy_(sd["critic_target.qf0.2.bias"])
    trainer.target_critic1.net[4].weight.data.copy_(sd["critic_target.qf0.4.weight"])
    trainer.target_critic1.net[4].bias.data.copy_(sd["critic_target.qf0.4.bias"])

    trainer.target_critic2.net[0].weight.data.copy_(sd["critic_target.qf1.0.weight"])
    trainer.target_critic2.net[0].bias.data.copy_(sd["critic_target.qf1.0.bias"])
    trainer.target_critic2.net[2].weight.data.copy_(sd["critic_target.qf1.2.weight"])
    trainer.target_critic2.net[2].bias.data.copy_(sd["critic_target.qf1.2.bias"])
    trainer.target_critic2.net[4].weight.data.copy_(sd["critic_target.qf1.4.weight"])
    trainer.target_critic2.net[4].bias.data.copy_(sd["critic_target.qf1.4.bias"])
    print("Target critic weights copied.")

    # Save with metadata so the inference node knows the architecture
    import os
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    torch.save(
        {
            "actor": actor.state_dict(),
            "critic1": critic1.state_dict(),
            "critic2": critic2.state_dict(),
            "target_critic1": trainer.target_critic1.state_dict(),
            "target_critic2": trainer.target_critic2.state_dict(),
            "actor_optim": trainer.actor_optim.state_dict(),
            "critic1_optim": trainer.critic1_optim.state_dict(),
            "critic2_optim": trainer.critic2_optim.state_dict(),
            "log_alpha": trainer.log_alpha.data,
            "alpha_optim": trainer.alpha_optim.state_dict(),
            "total_updates": 0,
            # Metadata so inference node knows which architecture to use
            "hidden1": 256,
            "hidden2": 256,
            "num_lidar": num_lidar,
            "sb3_converted": True,
        },
        out_path,
    )
    print(f"Checkpoint saved to {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert SB3 SAC to custom checkpoint")
    parser.add_argument("--sb3-path", required=True, help="Path to SB3 .zip model")
    parser.add_argument("--out", default="sac/sac_checkpoint.pth", help="Output path")
    parser.add_argument("--num-lidar", type=int, default=181)
    args = parser.parse_args()
    convert(args.sb3_path, args.out, args.num_lidar)
