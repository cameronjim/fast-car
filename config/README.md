# config

Single source of physical truth for the vehicle: `vehicle_params.yaml` and its JSON schema
live here, plus one directory per track under `tracks/<venue>_<layout>/` holding that
track's map, raceline, and timing-gate position. Nothing elsewhere in the repo may
hand-write a physical constant that belongs here; code consumes these values only through
generated bindings. See `claude-docs/02-repo-layout.md` and `claude-docs/06-vehicle-params.md`.
The params file and schema themselves are added by roadmap task 0.7, not this skeleton.
