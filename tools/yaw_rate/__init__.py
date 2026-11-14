"""Yaw rate from scan matching, measured offline against the map pose (roadmap P2.2).

Nothing here runs on the car. See docs/notes/yaw-rate-scan-matching-2026-11-13.md and section 3.4
of docs/notes/preregistration-pnp-v0.2.md. Pure numpy apart from `bagio` (mcap) and `report`
(matplotlib, optional).

Modules: `bagio` (the topics of a bag as numpy arrays, cached), `reference` (yaw change between
consecutive /pose messages, jump filter), `signals` (integrating a rate or a step series over
arbitrary intervals, closure noise), `delay` (cross-correlation lag), `matching` (scan-to-scan
ICP with the quality flags), `report` (tables, JSON, plots).
"""
