#!/usr/bin/env python3
"""Find the racer on the LAN and open its web UI. Mac-side helper, standard library only.

    python3 tools/racer_app/find_car.py            # find it, print the URL, open the browser
    python3 tools/racer_app/find_car.py --no-open  # just print the URL

Order: ask mDNS for racer-car.local (the Jetson advertises it with avahi), then, if that finds
nothing, scan the local /24 for something that answers GET /health on port 8080 with the
racer agent's JSON. The scan only trusts a host whose /health says name == "racer-agent".
"""

from __future__ import annotations

import argparse
import concurrent.futures
import ipaddress
import json
import socket
import sys
import urllib.error
import urllib.request
import webbrowser
from collections.abc import Callable, Iterable

DEFAULT_NAME = "racer-car.local"
DEFAULT_PORT = 8080
AGENT_NAME = "racer-agent"


def probe(host: str, port: int = DEFAULT_PORT, timeout: float = 0.6) -> dict | None:
    """Return the agent's /health JSON if `host:port` is a racer agent, else None."""
    url = f"http://{host}:{port}/health"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            data = json.loads(resp.read(4096).decode("utf-8"))
    except (OSError, ValueError, urllib.error.URLError):
        return None
    if isinstance(data, dict) and data.get("name") == AGENT_NAME:
        return data
    return None


def resolve_mdns(name: str = DEFAULT_NAME) -> list[str]:
    """Resolve a .local name through the system resolver (macOS speaks mDNS natively)."""
    try:
        infos = socket.getaddrinfo(name, None, socket.AF_INET, socket.SOCK_STREAM)
    except OSError:
        return []
    seen: list[str] = []
    for info in infos:
        addr = info[4][0]
        if addr not in seen:
            seen.append(addr)
    return seen


def local_ipv4() -> str | None:
    """This machine's LAN address (the source address a packet to the LAN would use)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))  # no packet is sent for a UDP connect
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def subnet_hosts(local_ip: str, prefix: int = 24) -> list[str]:
    net = ipaddress.ip_network(f"{local_ip}/{prefix}", strict=False)
    return [str(h) for h in net.hosts() if str(h) != local_ip]


def scan(
    hosts: Iterable[str],
    port: int = DEFAULT_PORT,
    timeout: float = 0.4,
    workers: int = 64,
    prober: Callable[..., dict | None] = probe,
) -> list[str]:
    """Hosts that answer as a racer agent, in address order."""
    host_list = list(hosts)
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(lambda h: (h, prober(h, port, timeout)), host_list))
    return [h for h, found in results if found]


def find_car(
    name: str = DEFAULT_NAME,
    port: int = DEFAULT_PORT,
    resolver: Callable[[str], list[str]] = resolve_mdns,
    scanner: Callable[[], list[str]] | None = None,
    prober: Callable[..., dict | None] = probe,
    log: Callable[[str], None] = lambda _: None,
) -> str | None:
    """Return the UI URL, or None. mDNS first, LAN scan as the fallback."""
    for addr in resolver(name):
        log(f"mDNS: {name} is {addr}")
        if prober(addr, port):
            return f"http://{name}:{port}/"
    log("mDNS found nothing that answers /health; scanning the local network")
    found = (scanner or default_scanner(port))()
    if found:
        log(f"scan: racer agent at {found[0]}")
        return f"http://{found[0]}:{port}/"
    return None


def default_scanner(port: int) -> Callable[[], list[str]]:
    def run() -> list[str]:
        ip = local_ipv4()
        return scan(subnet_hosts(ip), port) if ip else []

    return run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Find the racer and open its web UI.")
    parser.add_argument("--no-open", action="store_true", help="print the URL, do not open it")
    parser.add_argument("--name", default=DEFAULT_NAME)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args(argv)
    url = find_car(args.name, args.port, log=lambda m: print(m, file=sys.stderr))
    if url is None:
        print(
            "Could not find the car. Is it powered, on this Wi-Fi, and is racer-agent running?\n"
            "On the Jetson: systemctl status racer-agent",
            file=sys.stderr,
        )
        return 1
    print(url)
    if not args.no_open:
        webbrowser.open(url)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
