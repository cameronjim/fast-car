# Learning-based control (BC + SAC)

This document explains the `learned_control` package: how the driving data is
prepared, how the behavioural cloning model is trained, and how Soft Actor-Critic
refines it. It assumes you know basic deep learning and some reinforcement
learning, and fills in the specifics of this implementation.

The short version: collecting enough real driving data to train a reinforcement
learning policy from scratch on a physical car is not practical, so the policy is
first trained to copy recorded driving with behavioural cloning, then improved in
the simulator with Soft Actor-Critic that starts from the cloned policy.

Contents:

- [Data and preprocessing](#data-and-preprocessing)
- [Behavioural cloning](#behavioural-cloning)
- [Soft Actor-Critic](#soft-actor-critic)
- [Warm-starting SAC from BC](#warm-starting-sac-from-bc)
- [Reward function](#reward-function)
- [Online training loop](#online-training-loop)

## Data and preprocessing

The input to every model is a single LiDAR scan. The output is two numbers:
steering angle and speed.

`preprocessing/extract_dataset.py` reads a recorded ROS 2 bag and lines up the
`/scan`, `/drive`, and odometry messages by timestamp, producing one CSV row per
synchronized sample.

`preprocessing/preprocess.py` then cleans and prepares that CSV:

- Invalid LiDAR values (infinity, NaN) are replaced with the maximum range and
  clipped to a sensible range.
- Stationary samples (speed near zero) are dropped, since they teach the model
  nothing useful about driving.
- The scan is downsampled by keeping every 6th ray, which takes the 1080-point
  scan down to 181 features. The inference nodes use the same step so the live
  scan matches the training data.
- The data is mirrored left-to-right for augmentation: each scan is reversed and
  its steering angle is negated. This doubles the data and removes any bias
  toward turning one direction.
- Inputs and outputs are min-max normalized to roughly `[0, 1]`. The scale and
  offset for both the LiDAR and the actions are saved to `processed/scalers.npz`.

Normalization and its inverse are simple affine maps. To normalize a raw value:

```
x_norm = x * scale + offset
```

and to recover a physical action from a normalized one:

```
action = (action_norm - action_offset) / action_scale
```

The inference nodes load these saved scalers so the model always sees inputs on
the same scale it was trained on, and so its outputs can be turned back into real
steering and speed commands.

## Behavioural cloning

Behavioural cloning (BC) is plain supervised learning: given a scan, predict the
steering and speed a human would have used.

The model (`bc/model.py`) is a small multilayer perceptron:

```
181 LiDAR features -> Linear(256) -> ReLU -> Linear(128) -> ReLU -> Linear(2)
```

The two outputs are normalized steering and speed.

Training (`bc/train.py`) uses mean squared error between the predicted action and
the recorded action:

```
L_BC = mean( (predicted_action - recorded_action)^2 )
```

It uses the Adam optimizer (default learning rate `1e-3`), an 80/20 train and
validation split, and saves the weights whenever validation loss improves.

BC is a good baseline but it has a known weakness: it only knows states that
appeared in the training data. Once the car drifts into a situation the human
never demonstrated, small errors compound and there is nothing in the data to
correct them. That is the gap SAC is meant to close.

## Soft Actor-Critic

Soft Actor-Critic (SAC) is an off-policy reinforcement learning algorithm. It
learns by trial and error in the simulator, maximizing reward while also keeping
the policy as random as it can get away with. That extra randomness (entropy)
keeps exploration alive and makes training more stable.

There are three kinds of networks (`sac/model.py`):

- **Actor** (the policy). A shared trunk `181 -> 256 -> 128` feeds two heads that
  output the mean and log standard deviation of a Gaussian over actions. The log
  standard deviation is clamped to `[-20, 2]` for numerical stability.
- **Two critics.** Each takes the state and action together, `(181 + 2) -> 256
  -> 128 -> 1`, and estimates the value `Q(s, a)`. Using two and taking the
  smaller value is a standard trick to avoid overestimating value.
- **Two target critics.** Slow-moving copies of the critics used to compute
  training targets, updated by Polyak averaging.

**Action sampling and squashing.** Actions must end up in `[0, 1]`. The actor
samples from its Gaussian, squashes the sample through `tanh` to `(-1, 1)`, then
rescales to `(0, 1)`:

```
x ~ Normal(mean, std)
y = tanh(x)
action = (y + 1) / 2
```

Squashing changes the probability density, so the log-probability needs a
correction term (summed over the action dimensions):

```
log_prob(action) = log Normal(x) - sum( log(1 - y^2 + 1e-6) )
```

**Critic target.** For a sampled batch of transitions `(s, a, r, s', done)`, the
next action `a'` is drawn from the current actor, and the target value uses the
smaller of the two target critics minus an entropy term:

```
target = r + gamma * (1 - done) * ( min(Qt1(s', a'), Qt2(s', a')) - alpha * log_prob(a' | s') )
```

Each critic is trained to match this target with mean squared error, and
gradients are clipped to norm 1:

```
L_critic = mean( (Q(s, a) - target)^2 )
```

**Actor update.** The actor is trained to pick actions the critics value highly
while staying random, again using the smaller critic:

```
L_actor = mean( alpha * log_prob(a | s) - min(Q1(s, a), Q2(s, a)) )
```

**Entropy temperature.** The weight `alpha` on the entropy term is tuned
automatically toward a target entropy of `-action_dim` (here `-2`):

```
L_alpha = -mean( log_alpha * (log_prob + target_entropy) )
```

**Target network update.** After each step the target critics are moved a small
fraction `tau` toward the live critics (Polyak averaging):

```
target_params = (1 - tau) * target_params + tau * params
```

Default hyperparameters: discount `gamma = 0.99`, Polyak `tau = 0.005`, batch
size 256, replay buffer 100000, actor and entropy learning rate `1e-4` to `3e-4`,
critic learning rate `3e-4`.

## Warm-starting SAC from BC

Training SAC from random weights would throw away everything BC already learned.
Instead the SAC actor is initialized from the BC model (`SACActorNet.from_bc`).

The two hidden layers are copied directly. The output head needs a small
adjustment because the two models parameterize actions differently: BC outputs a
normalized action `y` in `[0, 1]` directly, while the SAC actor outputs a
pre-squash mean that later becomes `(tanh(mean) + 1) / 2`. Around the usual
operating point `y` near 0.5, `atanh(2y - 1)` is close to linear, so the BC head
maps into the pre-squash space with `mean ~= 2y - 1`. That is why the copied
output weights are scaled by 2 and the bias is shifted by `-1`. The log standard
deviation head is initialized to a small constant so the policy starts nearly
deterministic and close to the BC policy.

## Reward function

The reward (`sac/reward.py`) is computed every step from the raw LiDAR, the
speed, the steering angle, and whether the episode ended:

```
reward  =  0.1                                    # survival bonus, per step
        +  0.1 * speed                            # encourage forward progress
        -  2.0 * (0.5 - min_range)  if min_range < 0.5   # penalize being near a wall
        -  0.8 * |steering - prev_steering|       # penalize jerky steering
        -  50.0                     if crashed     # large crash penalty
```

The pieces pull in the obvious directions: stay alive, go faster, keep away from
walls, steer smoothly, and never crash. The crash penalty is large so the policy
strongly prefers finishing a lap over taking risks.

## Online training loop

`sac_train_node` runs SAC live in the simulator. Each LiDAR scan produces an
action, the reward is computed, and the transition is stored in the replay
buffer. The loop ramps up in phases controlled by parameters:

- `warmup_steps`: early on, actions come from the warm-started policy while the
  buffer fills.
- `learning_starts`: after this many steps the critics begin updating.
- `actor_learning_starts`: the actor and entropy temperature start updating later
  than the critics, so the critics are somewhat reliable before the actor chases
  them.
- `update_every`: how often a gradient step runs.
- `save_every`: how often a checkpoint is written. The node also saves a separate
  best checkpoint when an episode goes especially well.

Early in training the actor is also pulled toward the BC policy with a
regularization term added to the actor loss:

```
L_actor_total = L_actor + bc_reg_weight * mean( (action - bc_action)^2 )
```

The weight decays over `bc_reg_decay_steps`, so the policy leans on BC at first
and is gradually allowed to depart from it as it learns to drive better than the
demonstrations. When the car crashes, the node resets the simulated car to a
start pose and continues. The result is a single policy used for both the
simulator and the physical car, loaded for inference by `sac_demo_node`.
