"""Shared, race-safe replacement for a bare ``rclpy.spin(node)`` in a node's ``main()``.

GitHub issue 50. rclpy installs its own SIGINT handler that shuts the context down. A SIGINT that
lands while rclpy is taking a message surfaces from ``rclpy.spin`` as
``RuntimeError: Unable to convert call argument to Python object`` (the take returned after the
context was already down), the node exits 1 and launch_testing's exit-code check fails. It is a
shutdown race, not a node bug, so every Python node's ``main()`` uses this one helper instead of
repeating the guard.

``rclpy`` is imported inside the function so this module imports (and is unit-tested) without ROS.
"""

from __future__ import annotations


def spin_until_shutdown(node: object) -> None:
    """Run ``rclpy.spin(node)``, treating a RuntimeError during shutdown as a clean exit.

    The RuntimeError is swallowed only when ``rclpy.ok()`` is False afterwards, i.e. rclpy's own
    SIGINT handler had already shut the context down. With the context still up the error is a
    real one and is re-raised unchanged. Other exceptions (KeyboardInterrupt, ...) propagate
    untouched; the caller keeps its own ``except`` and ``finally`` (destroy the node, and call
    ``rclpy.shutdown()`` only if ``rclpy.ok()``, since rclpy's handler may have done it already).
    """
    import rclpy

    try:
        rclpy.spin(node)
    except RuntimeError:
        if rclpy.ok():
            raise
