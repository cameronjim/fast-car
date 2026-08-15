"""Actual model loading: the one deploy-time-only piece of this package.

`racer_policy.contract.load_contract` and `racer_policy.verify.verify_against_environment`
never need `torch` -- checksum verification hashes raw bytes, and every comparison in this
package is over plain dataclasses. Only `load_model` needs the real weights loaded into a
runnable model, which does need `torch`. Per claude-docs/08-learning.md ("Inference target:
50 Hz on the Jetson ... Python initially") and the S.5 task scope, `torch` is imported
LAZILY inside this function, not at module import time and not as a declared dependency in
pyproject.toml, so:

- the L1 CI job (claude-docs/12-testing.md) that exercises the contract loader and refusal
  paths never needs torch installed, and
- importing `racer_policy` anywhere (including from the eventual ROS-free unit tests for
  policy_node's decision logic, roadmap 5.2) stays cheap.

Call `load_contract` (and, before deploying, `verify_against_environment`) FIRST; this
function does not re-verify the checksum or re-run the environment checks -- it only turns
an already-trusted `Contract` into a loaded model object.
"""

from __future__ import annotations

from typing import Any

from racer_policy.contract import Contract
from racer_policy.errors import MissingTorchDependencyError


def load_model(contract: Contract) -> Any:
    """Load the TorchScript policy artifact `contract` points at.

    Raises `MissingTorchDependencyError` if `torch` is not installed in the current
    environment -- expected and fine in L1 CI, a hard stop in any environment meant to
    actually run inference (claude-docs/08-learning.md: deploy-time only).
    """
    try:
        import torch
    except ImportError as exc:
        raise MissingTorchDependencyError(
            "torch is required to load a policy model (racer_policy.model.load_model) but "
            "is not installed in this environment. This is expected in the L1 CI job "
            "(claude-docs/12-testing.md) -- torch is a deploy-time-only dependency "
            "(claude-docs/08-learning.md). Install torch in the deploy environment before "
            "calling load_model."
        ) from exc

    return torch.jit.load(str(contract.policy_path), map_location="cpu")
