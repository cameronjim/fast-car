# Roadmap, Phase 5: applications on top of the racer

Tracked copy of the Phase 5 section of the (gitignored) roadmap, added 2026-10-16 at the owner's request. Same repo; the apps live under `ros_ws/src/racer_apps/` when they start. Nothing here bypasses the safety mux or the safety node.

Spin-offs the owner wants after the racing thesis; they reuse the stack (LiDAR driving, safety node, odometry, parking/approach primitives, cameras) and never bypass the safety mux.
- [~] 5.1 Camera perception (software landed 2026-10-23, `docs/notes/perception-2026-10-23.md`; no weights, no image build, nothing run on the car): YOLO-class detector on the Jetson (PyTorch; TensorRT stays out of scope unless lifted), trained off-car on bags from the one CSI module + the ELP USB camera; publishes detections with bearing/range by fusing with the LiDAR scan.
- [ ] 5.2 Target behaviour: detect a person wearing a chosen shirt (colour/logo, OpenCV or the detector), drive toward them (gap follow + the approach primitive from park_node), stop at a set distance, trigger an actuator. First actuator: the water squirter (parts researched 2026-09-21, own battery pack, isolated from the mux rail).
- [ ] 5.3 "Spiderbot" patrol: waypoint patrol on the localization stack (2.4/2.5), pose estimation (YOLO-pose) with a temporal rule for punches/kicks, then the actuator ("web" = squirter or a foam launcher), with an operator confirm step.
- [ ] 5.4 Care-bot: detect a person (optionally fallen, via pose), drive to them, stop, deliver a payload tray, alert over the network.
- [ ] 5.5 Learned driving: the thesis residual policy (Phase 4) first; then imitation/diffusion policy from LiDAR + camera bags collected by gap_follow laps.
- [ ] 5.6 Follow-me: lock onto one person (detector + LiDAR range) and shadow them at a set distance.
- [ ] 5.7 Night watch: patrol a loop on the map, log and photograph anything new versus the stored LiDAR map.
- [ ] 5.8 Summon: a phone or voice trigger drives the car to the caller (needs localization and the approach primitive).

