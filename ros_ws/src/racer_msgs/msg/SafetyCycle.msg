# racer_safety/safety_node publishes one of these per gate cycle on /safety/cycle when its
# `publish_cycle_record` parameter is true (default false: the topic does not exist). It is
# INSTRUMENTATION: a record of which /drive_raw the gate had latched at each cycle and when that
# command reached the node, so the timing between the planner and /drive can be measured
# (brief 45, owner decision 2026-11-14 #1). It never feeds the gate and nothing may depend on
# it for safety. Unlike /safety/events (transitions only) this is a per-cycle stream, so a
# consumer counts cycles, not interventions.
#
# Published AFTER the cycle's /drive and /safety/events, from values the cycle already computed,
# so the record never delays or alters the gated output. QoS: depth 10, best effort (sensor
# data).

# The cycle stamp: the same value the /drive message of this cycle carries (the node's ROS
# clock; see CLOCK POLICY at the top of safety_node.cpp). Join /drive to this record on this
# field.
builtin_interfaces/Time stamp

# False until the first /drive_raw has been received; the gate then judges a zero command and
# the watchdog is tripped. While false, drive_raw_stamp is zero, drive_raw_age_s is -1.0 and
# drive_raw_count is 0.
bool has_command

# Header stamp of the /drive_raw message the gate had latched at this cycle (the publisher's
# stamp, not the node's receipt time); zero when has_command is false.
builtin_interfaces/Time drive_raw_stamp

# Seconds the latched command had waited at this cycle, on safety_node's steady (monotonic)
# clock: exactly the `drive_raw_age_s` the gate received. The receipt time of the latched
# command is therefore (cycle time on that clock) minus this value. Infinity (no command yet)
# is encoded as -1.0.
float64 drive_raw_age_s

# /drive_raw messages received since the node started. A cycle that did not advance this
# counter reused the previous command (a repeated latch); an advance by more than 1 means the
# node received several commands between two cycles and only the last one was judged (a skipped
# latch).
uint32 drive_raw_count

# Seconds between this gate cycle and the previous one on the steady clock: exactly the `dt_s`
# the gate received (the nominal control period on the first cycle).
float64 cycle_dt_s
