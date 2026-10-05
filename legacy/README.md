# legacy/

Code kept for reference, not built, not tested, not run on the car.

- `f1tenth-autonomous-racing/`: the ROS 2 Foxy, Python, sim-only project this repo grew out
  of (merged by git subtree, history preserved). Its reactive controllers are being ported
  into `ros_ws/src/racer_control` in C++ (GitHub issue 26); its RL training code stays here
  untouched. Nothing under this directory is imported by the live stack.
