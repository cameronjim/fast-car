"""The named refusals of the model-bundle loader (config/pnp/README.md, the refusal table).

Every refusal is its own class, all subclasses of `BundleRefused`. There is no override and no
downgrade to a warning: a node that gets one of these exits non-zero.
"""

from __future__ import annotations


class BundleRefused(Exception):
    """Base of every refusal. The message says what differed."""


class ManifestMissing(BundleRefused):
    """No manifest.yaml, or it does not parse."""


class ManifestInvalid(BundleRefused):
    """A missing field, an extra top-level field, or a wrong type."""


class BundleSchemaVersionMismatch(BundleRefused):
    """The major version of bundle_schema_version is not the one this loader reads."""


class KindMismatch(BundleRefused):
    """The bundle's kind is not the kind this node deploys."""


class PayloadMissing(BundleRefused):
    """A payload file is absent."""


class ChecksumMismatch(BundleRefused):
    """The SHA-256 of a payload file differs from the manifest."""


class VehicleParamsMismatch(BundleRefused):
    """schema_version or sysid_session_id differs from the live vehicle_params."""


class IoContractMismatch(BundleRefused):
    """An io value differs from what this node implements."""
