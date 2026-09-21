#!/usr/bin/env python3
"""Idempotently point extlinux.conf's OVERLAYS line at the racer pinmux overlay.

Called by install.sh; not meant to be run on its own. Takes the path to
extlinux.conf and the absolute path of the .dtbo to reference.

It also drops /boot/jetson-io-hdr40-user-custom.dtbo from the line, because this
overlay replaces it -- see the comment in install.sh and README.md's "Ordering
and coexistence".
"""

import re
import sys

CONFLICT = "/boot/jetson-io-hdr40-user-custom.dtbo"


def main(path: str, dtbo: str) -> int:
    with open(path) as fh:
        lines = fh.read().splitlines()

    changed = False
    found = False

    for i, line in enumerate(lines):
        m = re.match(r"^(\s*)OVERLAYS(\s+)(.*)$", line)
        if not m:
            continue
        found = True
        indent, sep, value = m.groups()
        items = [e for e in re.split(r"[,\s]+", value.strip()) if e]
        kept = [e for e in items if e != CONFLICT]
        if len(kept) != len(items):
            print(f"    removing conflicting overlay {CONFLICT}")
        if dtbo not in kept:
            kept.append(dtbo)
        replacement = f"{indent}OVERLAYS{sep}{','.join(kept)}"
        if replacement != line:
            lines[i] = replacement
            changed = True

    if not found:
        anchor = None
        for i, line in enumerate(lines):
            if re.match(r"^\s*(LINUX|INITRD|FDT|APPEND)\b", line):
                anchor = i
        if anchor is None:
            print(
                "no LINUX/INITRD/FDT/APPEND line found; refusing to guess where OVERLAYS goes",
                file=sys.stderr,
            )
            return 1
        indent = re.match(r"^(\s*)", lines[anchor]).group(1) or "\t"
        lines.insert(anchor + 1, f"{indent}OVERLAYS {dtbo}")
        changed = True

    if changed:
        with open(path, "w") as fh:
            fh.write("\n".join(lines) + "\n")
        print("    extlinux.conf updated")
    else:
        print("    extlinux.conf already correct; unchanged")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: _edit_extlinux.py <extlinux.conf> <dtbo-path>", file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
