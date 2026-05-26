# scratch: why the same gate seeds fail for every policy

import numpy as np
from stable_baselines3 import SAC

from f1rl.envs import build_env, load_config

FAILING = (0, 11, 13, 14)
PASSING = (1, 5)

cfg = load_config("configs/sac_m7_versus.yaml")
env = build_env(cfg, seed_offset=0)
sim = env.unwrapped.sim
model = SAC.load("/tmp/m7_240k.zip", device="cpu")

for seed in FAILING + PASSING:
    obs, info = env.reset(seed=seed)
    spawned_in_contact = list(getattr(sim, "_spawned_in_contact", None) or [])
    poses = np.asarray(sim.state.poses)
    print(
        f"\nseed {seed}: spawn contact {spawned_in_contact} spawn gap {info['gap_m']:.2f} "
        f"spawn dist {info['opponent_distance_m']:.2f} opp cap {info['opponent_speed_cap_mps']:.2f}"
    )
    trace = []
    terminated = truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, info = env.step(action)
        std = sim.state.standard_state
        trace.append(
            (
                info["sim_time"],
                float(std[0][3]),
                float(std[1][3]),
                info["opponent_distance_m"],
                info["gap_m"],
                float(info["command"][1]),
            )
        )
    tag = "collision" if info["is_collision"] else ("passed" if info["overtaken"] else "stuck")
    print(f"  ended {tag} at {info['sim_time']:.2f}s, opponent crashed {info['opponent_collision']}")
    for row in trace[-6:]:
        print(
            f"    t {row[0]:5.2f} ego_v {row[1]:5.2f} opp_v {row[2]:5.2f} dist {row[3]:5.2f} "
            f"gap {row[4]:6.2f} cmd_v {row[5]:5.2f}"
        )
env.close()
