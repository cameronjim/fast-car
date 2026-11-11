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
