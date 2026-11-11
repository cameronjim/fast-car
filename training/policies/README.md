# Policy pointers

Policy directories are not committed (the repo commits no model binaries). They live outside
the repo, under `car-data/policies/<name>/`, and are copied or symlinked into
`<repo>/data/policies/<name>/` on the machine that runs the car (`data/` is gitignored). The
racer app looks there for a directory whose `contract.yaml` passes `racer_policy.load_contract`.

Each subdirectory here holds the small text artifacts of one policy: `contract.yaml`,
`policy_meta.yaml`, the held-out summary and the latency measurement. The binary is identified
by the checksum inside `contract.yaml` (`policy.checksum_sha256`).

| Name | Checksum of policy.pt | Note |
|---|---|---|
| `v0-2026-11-11` | `3a7fd7fe4ad9b17516ad3039c2e3766477de94378b2da1c1b18940b473ee938b` | v0, pre-sysid, not the thesis result. See `docs/notes/residual-v0-2026-11-11.md`. |
| `v1-2026-11-11` | `f0b4a5ed313279216b151955e77e06fc2bff4b52a5d330f5b6a94171503ec792` | v1, corrected sim, `vehicle_params` 0.14.0, no overrides. NO WIN over the tracker in sim (slightly slower, same completion); a pipeline and deployment-contract artifact, not a result. See `docs/notes/residual-v1-2026-11-11.md`. The v0 directory is now refused by the loader (it names 0.13.0). |
