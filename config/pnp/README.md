# Plug-and-Play model bundles

A bundle is a directory holding `manifest.yaml` plus the payload files it lists (weights,
golden rollouts). `bundle.schema.json` is the JSON Schema (draft 2020-12) every manifest must
satisfy. The manifest ties the model to a git commit, a training config hash, a vehicle_params
version and a SHA-256 for each payload file. Bundles live under `data/pnp/models/<model_id>/`
and are never committed; only the small fixtures under `fixtures/` are.

Every loader refuses on mismatch, with no override flag and no warning path:

| Error | Condition |
|---|---|
| `ManifestMissing` | no `manifest.yaml`, or it does not parse |
| `ManifestInvalid` | missing field, extra top-level field, wrong type |
| `BundleSchemaVersionMismatch` | major of `bundle_schema_version` is not 1 |
| `KindMismatch` | `kind` is not what the node deploys |
| `PayloadMissing` | a listed payload file is absent |
| `ChecksumMismatch` | SHA-256 of a payload file differs from the manifest |
| `VehicleParamsMismatch` | `schema_version` or `sysid_session_id` differs from the live vehicle_params |
| `IoContractMismatch` | any `io` value differs from what the node implements |
| `GoldenRolloutMismatch` | dynamics only: the loader's model does not reproduce the golden rollouts within 1e-4 |
| `ValidityExceeded` | dynamics only: requested speed cap is above `io.validity.v_max_mps` |
