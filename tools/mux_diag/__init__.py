"""Bench tooling for reading the safety_mux diagnostic firmware's serial output.

Not part of the control path -- run by a person at the bench, on the Jetson, with a Pico
running the DIAG_BUILD firmware from `docs/notes/mux-diagnostic-build.md` plugged into USB.
See `read_mux_diag.py` for the CLI.
"""
