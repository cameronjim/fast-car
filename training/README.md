# training

Residual-policy training code. `racer_train` holds the SAC/PPO training package that learns
a residual on top of the classical controller; `envelope` is the bounds/rate-limit/OOD
fallback library, shared unmodified between the training environment and the on-vehicle
`racer_policy` deploy node so the two never diverge; `configs` holds experiment configs that
get hashed into the deployment contract. See `claude-docs/02-repo-layout.md` and
`claude-docs/08-learning.md`.
