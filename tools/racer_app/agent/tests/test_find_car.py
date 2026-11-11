"""The Mac-side finder (tools/racer_app/find_car.py): mDNS first, LAN scan as the fallback."""

from __future__ import annotations

import importlib.util
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "find_car", Path(__file__).resolve().parents[2] / "find_car.py"
)
find_car = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(find_car)


def serve(body: bytes, status: int = 200):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(status)
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


@pytest.fixture
def agent():
    server = serve(json.dumps({"ok": True, "name": "racer-agent", "version": "0.1.0"}).encode())
    yield server.server_address[1]
    server.shutdown()


def test_probe_accepts_the_racer_agent(agent):
    assert find_car.probe("127.0.0.1", agent)["name"] == "racer-agent"


def test_probe_rejects_other_servers_and_garbage():
    other = serve(json.dumps({"ok": True, "name": "something-else"}).encode())
    junk = serve(b"<html>not json</html>")
    try:
        assert find_car.probe("127.0.0.1", other.server_address[1]) is None
        assert find_car.probe("127.0.0.1", junk.server_address[1]) is None
    finally:
        other.shutdown()
        junk.shutdown()


def test_probe_of_a_closed_port_is_none():
    assert find_car.probe("127.0.0.1", 1, timeout=0.2) is None


def test_scan_returns_only_hosts_that_answer_as_the_agent(agent):
    hosts = ["127.0.0.1", "127.0.0.2"]

    def prober(host, port, timeout):
        return find_car.probe(host, port, timeout) if host == "127.0.0.1" else None

    assert find_car.scan(hosts, agent, prober=prober) == ["127.0.0.1"]


def test_subnet_hosts_skips_our_own_address():
    hosts = find_car.subnet_hosts("192.168.1.20")
    assert len(hosts) == 253 and "192.168.1.20" not in hosts
    assert hosts[0] == "192.168.1.1" and hosts[-1] == "192.168.1.254"


def test_mdns_wins_when_it_resolves_and_answers():
    calls = []
    url = find_car.find_car(
        resolver=lambda name: ["10.0.0.226"],
        scanner=lambda: calls.append("scan") or [],
        prober=lambda host, port, *a: {"name": "racer-agent"},
    )
    assert url == "http://racer-car.local:8080/"
    assert calls == []


def test_falls_back_to_the_scan_when_mdns_is_empty_or_wrong():
    for resolver, prober in (
        (lambda name: [], lambda *a: None),
        (lambda name: ["10.0.0.9"], lambda *a: None),  # resolves, but not our agent
    ):
        url = find_car.find_car(
            resolver=resolver, scanner=lambda: ["192.168.1.77", "192.168.1.78"], prober=prober
        )
        assert url == "http://192.168.1.77:8080/"


def test_nothing_found_is_none():
    assert find_car.find_car(resolver=lambda n: [], scanner=list) is None


def test_log_callback_reports_the_route_taken():
    lines = []
    find_car.find_car(resolver=lambda n: [], scanner=lambda: ["1.2.3.4"], log=lines.append)
    assert any("scanning" in line for line in lines) and any("1.2.3.4" in line for line in lines)


def test_resolve_mdns_swallows_errors_and_dedupes(monkeypatch):
    monkeypatch.setattr(
        find_car.socket, "getaddrinfo", lambda *a, **k: (_ for _ in ()).throw(OSError())
    )
    assert find_car.resolve_mdns("nope.local") == []
    infos = [
        (2, 1, 6, "", ("10.0.0.1", 0)),
        (2, 1, 6, "", ("10.0.0.1", 0)),
        (2, 1, 6, "", ("10.0.0.2", 0)),
    ]
    monkeypatch.setattr(find_car.socket, "getaddrinfo", lambda *a, **k: infos)
    assert find_car.resolve_mdns() == ["10.0.0.1", "10.0.0.2"]


def test_local_ipv4_is_an_address_or_none():
    ip = find_car.local_ipv4()
    assert ip is None or ip.count(".") == 3


def test_default_scanner_scans_the_local_subnet(monkeypatch):
    monkeypatch.setattr(find_car, "local_ipv4", lambda: "192.168.5.9")
    seen = {}
    monkeypatch.setattr(
        find_car, "scan", lambda hosts, port: seen.update(n=len(hosts), p=port) or ["x"]
    )
    assert find_car.default_scanner(8080)() == ["x"]
    assert seen == {"n": 253, "p": 8080}
    monkeypatch.setattr(find_car, "local_ipv4", lambda: None)
    assert find_car.default_scanner(8080)() == []


def test_main_prints_the_url_and_opens_it_unless_told_not_to(monkeypatch, capsys):
    monkeypatch.setattr(find_car, "find_car", lambda *a, **k: "http://racer-car.local:8080/")
    opened = []
    monkeypatch.setattr(find_car.webbrowser, "open", opened.append)
    assert find_car.main([]) == 0
    assert opened == ["http://racer-car.local:8080/"]
    assert capsys.readouterr().out.strip() == "http://racer-car.local:8080/"
    assert find_car.main(["--no-open"]) == 0
    assert len(opened) == 1


def test_main_says_what_to_check_when_the_car_is_not_found(monkeypatch, capsys):
    monkeypatch.setattr(find_car, "find_car", lambda *a, **k: None)
    assert find_car.main(["--no-open"]) == 1
    assert "systemctl status racer-agent" in capsys.readouterr().err
