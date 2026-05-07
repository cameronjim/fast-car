# sim

Simulation stack used for training and regression testing. `racer_gym` is the project's
fork/extension of `f1tenth_gym` carrying the vehicle-model upgrades (load transfer, Pacejka
tires, actuator dynamics, transport delay); `bridge` is the gym-to-ROS bridge ported to
Humble so the same control stack that runs on the car can run against the simulator. See
`claude-docs/02-repo-layout.md` and `claude-docs/07-sim-and-sysid.md` for the model and
bridge details.
