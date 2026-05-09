#!/usr/bin/env python3
# Browser-based UART monitor/control panel for Pi <-> ESP32 testing.

import argparse
import importlib.util
import json
import os
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_TESTING_SYSTEM_PYTHON"
DEFAULT_UART_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_HTTP_HOST = "0.0.0.0"
DEFAULT_HTTP_PORT = 8765
LOG_LIMIT = 200


def ensure_pyserial_runtime():
    if importlib.util.find_spec("serial") is not None:
        return

    if os.path.exists(SYSTEM_PYTHON) and os.environ.get(REEXEC_GUARD) != "1":
        os.environ[REEXEC_GUARD] = "1"
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])

    raise SystemExit(
        "pyserial is not installed for this Python interpreter.\n"
        f"Run this script with {SYSTEM_PYTHON} or install python3-serial."
    )


ensure_pyserial_runtime()

import serial


def timestamp():
    return time.strftime("%H:%M:%S")


def normalize_limit_state(line):
    text = line.strip().upper()
    if text in ("LIMIT:PRESSED", "PRESSED", "LIMIT=1", "LIMIT 1"):
        return "PRESSED"
    if text in ("LIMIT:RELEASED", "RELEASED", "LIMIT=0", "LIMIT 0", "OPEN"):
        return "RELEASED"
    return None


class SerialBridge:
    def __init__(self, uart_port, baud):
        self.uart_port = uart_port
        self.baud = baud
        self.link = None
        self.lock = threading.Lock()
        self.logs = deque(maxlen=LOG_LIMIT)
        self.next_log_id = 1
        self.last_limit_state = "UNKNOWN"
        self.last_rx_time = ""
        self.last_tx_time = ""
        self.last_error = ""
        self.stop_event = threading.Event()
        self.reader_thread = None

    def add_log(self, direction, message):
        entry = {
            "id": self.next_log_id,
            "time": timestamp(),
            "direction": direction,
            "message": message,
        }
        self.next_log_id += 1
        self.logs.append(entry)
        return entry

    def connect(self):
        with self.lock:
            self.last_error = ""
            if self.link is not None and self.link.is_open:
                return

            try:
                self.link = serial.Serial(self.uart_port, self.baud, timeout=0.2)
            except serial.SerialException as exc:
                self.link = None
                self.last_error = str(exc)
                self.add_log("SYS", f"Open failed: {exc}")
                return

            self.add_log("SYS", f"Connected to {self.uart_port} @ {self.baud}")
            self.stop_event.clear()
            self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
            self.reader_thread.start()

        self.send_line("STATUS?")

    def disconnect(self):
        with self.lock:
            self.stop_event.set()
            link = self.link
            self.link = None

        if link is not None:
            try:
                link.close()
            except serial.SerialException:
                pass

        with self.lock:
            self.add_log("SYS", "Disconnected")

    def reconnect(self):
        self.disconnect()
        time.sleep(0.1)
        self.connect()

    def send_line(self, text):
        payload = text.strip()
        if not payload:
            return False, "Empty command"

        with self.lock:
            link = self.link
            if link is None or not link.is_open:
                return False, "Serial port is not connected"

            try:
                link.write(f"{payload}\n".encode())
                link.flush()
            except serial.SerialException as exc:
                self.last_error = str(exc)
                self.add_log("SYS", f"Write failed: {exc}")
                return False, str(exc)

            self.last_tx_time = timestamp()
            self.add_log("TX", payload)
            return True, ""

    def reader_loop(self):
        while not self.stop_event.is_set():
            with self.lock:
                link = self.link

            if link is None:
                return

            try:
                raw = link.readline()
            except (serial.SerialException, OSError) as exc:
                with self.lock:
                    self.last_error = str(exc)
                    self.add_log("SYS", f"Read failed: {exc}")
                return

            if not raw:
                continue

            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            with self.lock:
                self.last_rx_time = timestamp()
                self.add_log("RX", line)
                state = normalize_limit_state(line)
                if state is not None:
                    self.last_limit_state = state

            if state is not None:
                self.send_line(f"ACK:{state}")

    def snapshot(self):
        with self.lock:
            connected = self.link is not None and self.link.is_open
            return {
                "connected": connected,
                "uart_port": self.uart_port,
                "baud": self.baud,
                "last_limit_state": self.last_limit_state,
                "last_rx_time": self.last_rx_time,
                "last_tx_time": self.last_tx_time,
                "last_error": self.last_error,
                "logs": list(self.logs),
            }


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pi UART Monitor</title>
  <style>
    :root {
      --bg: #f4efe6;
      --card: #fffaf2;
      --ink: #1c1a17;
      --muted: #6d665c;
      --accent: #0c6d62;
      --accent-2: #c4572b;
      --line: #d8ccb7;
      --good: #236b2c;
      --warn: #9f4f17;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Tahoma, sans-serif;
      background:
        radial-gradient(circle at top right, #f8cf9b 0, transparent 28%),
        linear-gradient(135deg, #f4efe6 0%, #efe4d3 100%);
      color: var(--ink);
    }
    .wrap {
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero {
      padding: 20px 24px;
      border: 1px solid var(--line);
      border-radius: 20px;
      background: rgba(255, 250, 242, 0.88);
      backdrop-filter: blur(8px);
      box-shadow: 0 18px 40px rgba(49, 37, 20, 0.08);
    }
    .hero h1 {
      margin: 0 0 8px;
      font-size: clamp(28px, 5vw, 44px);
      letter-spacing: 0.02em;
    }
    .hero p {
      margin: 0;
      color: var(--muted);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 18px;
      margin-top: 18px;
    }
    .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 12px 30px rgba(49, 37, 20, 0.06);
    }
    .label {
      color: var(--muted);
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .value {
      margin-top: 6px;
      font-size: 28px;
      font-weight: 700;
    }
    .value.good { color: var(--good); }
    .value.warn { color: var(--warn); }
    .controls {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 12px;
      margin-top: 18px;
    }
    button {
      border: none;
      border-radius: 14px;
      padding: 14px 16px;
      font-size: 15px;
      font-weight: 700;
      color: white;
      background: linear-gradient(135deg, var(--accent), #148579);
      cursor: pointer;
      box-shadow: 0 10px 20px rgba(12, 109, 98, 0.2);
    }
    button.alt {
      background: linear-gradient(135deg, var(--accent-2), #d17639);
      box-shadow: 0 10px 20px rgba(196, 87, 43, 0.18);
    }
    button.ghost {
      background: linear-gradient(135deg, #6a655d, #857e74);
      box-shadow: 0 10px 20px rgba(72, 67, 61, 0.16);
    }
    .raw-row {
      display: flex;
      gap: 12px;
      margin-top: 18px;
      flex-wrap: wrap;
    }
    input {
      flex: 1 1 280px;
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 14px 16px;
      font-size: 15px;
      background: #fff;
    }
    .log {
      margin-top: 18px;
      padding: 0;
      overflow: hidden;
    }
    .log-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
    }
    .log-window {
      background: #191613;
      color: #f5e7d2;
      border-radius: 16px;
      min-height: 320px;
      max-height: 480px;
      overflow: auto;
      padding: 14px;
      font-family: Consolas, "Courier New", monospace;
      font-size: 13px;
      line-height: 1.45;
    }
    .rx { color: #8ce6ce; }
    .tx { color: #ffd08d; }
    .sys { color: #f2a6a6; }
    .footer {
      margin-top: 14px;
      color: var(--muted);
      font-size: 14px;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>Pi UART Monitor</h1>
      <p>Live browser GUI for checking whether the ESP32 is receiving Pi commands and sending status back.</p>
    </section>

    <section class="grid">
      <div class="card">
        <div class="label">Serial Link</div>
        <div class="value" id="connected">Connecting...</div>
        <div class="footer" id="portInfo"></div>
      </div>
      <div class="card">
        <div class="label">Limit State</div>
        <div class="value" id="limitState">UNKNOWN</div>
        <div class="footer">Updates when the ESP32 sends LIMIT:PRESSED or LIMIT:RELEASED.</div>
      </div>
      <div class="card">
        <div class="label">Last RX</div>
        <div class="value" id="lastRx">--:--:--</div>
        <div class="footer">Any response here means the ESP is talking back to the Pi.</div>
      </div>
      <div class="card">
        <div class="label">Last Error</div>
        <div class="value warn" id="lastError">None</div>
        <div class="footer">If nothing is moving, check here first.</div>
      </div>
    </section>

    <section class="card">
      <div class="label">Quick Commands</div>
      <div class="controls">
        <button onclick="sendCommand('PING')">Ping ESP</button>
        <button onclick="sendCommand('STATUS?')">Read Status</button>
        <button class="alt" onclick="sendCommand('TEST HIGH')">Test High</button>
        <button class="alt" onclick="sendCommand('TEST LOW')">Test Low</button>
        <button class="ghost" onclick="reconnect()">Reconnect</button>
      </div>
      <div class="raw-row">
        <input id="rawInput" placeholder="Type any raw UART line here">
        <button onclick="sendRaw()">Send Raw</button>
      </div>
    </section>

    <section class="card log">
      <div class="log-head">
        <div class="label">Live Log</div>
        <div class="footer">TX shows commands from Pi. RX shows messages from ESP.</div>
      </div>
      <div class="log-window" id="logWindow"></div>
    </section>
  </div>

  <script>
    async function postJson(path, payload) {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload || {}),
      });
      return response.json();
    }

    async function sendCommand(command) {
      await postJson('/send', { command });
      await refresh();
    }

    async function sendRaw() {
      const input = document.getElementById('rawInput');
      const value = input.value.trim();
      if (!value) return;
      input.value = '';
      await sendCommand(value);
    }

    async function reconnect() {
      await postJson('/reconnect', {});
      await refresh();
    }

    function renderLog(entries) {
      const box = document.getElementById('logWindow');
      box.innerHTML = entries.map((entry) => {
        const klass = entry.direction === 'RX' ? 'rx' : (entry.direction === 'TX' ? 'tx' : 'sys');
        return `<div class="${klass}">[${entry.time}] ${entry.direction} ${entry.message}</div>`;
      }).join('');
      box.scrollTop = box.scrollHeight;
    }

    async function refresh() {
      const response = await fetch('/state');
      const state = await response.json();

      const connected = document.getElementById('connected');
      connected.textContent = state.connected ? 'Connected' : 'Disconnected';
      connected.className = `value ${state.connected ? 'good' : 'warn'}`;

      document.getElementById('portInfo').textContent = `${state.uart_port} @ ${state.baud} baud`;
      document.getElementById('limitState').textContent = state.last_limit_state;
      document.getElementById('lastRx').textContent = state.last_rx_time || '--:--:--';
      document.getElementById('lastError').textContent = state.last_error || 'None';
      renderLog(state.logs);
    }

    document.getElementById('rawInput').addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        sendRaw();
      }
    });

    refresh();
    setInterval(refresh, 700);
  </script>
</body>
</html>
"""


class UartRequestHandler(BaseHTTPRequestHandler):
    bridge = None

    def do_GET(self):
        if self.path == "/":
            self.respond_html(HTML_PAGE)
            return

        if self.path == "/state":
            self.respond_json(self.bridge.snapshot())
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if self.path == "/send":
            payload = self.read_json()
            command = payload.get("command", "")
            ok, error = self.bridge.send_line(command)
            self.respond_json({"ok": ok, "error": error})
            return

        if self.path == "/reconnect":
            self.bridge.reconnect()
            self.respond_json({"ok": True})
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format_str, *args):
        return

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            return json.loads(raw.decode() or "{}")
        except json.JSONDecodeError:
            return {}

    def respond_html(self, body):
        payload = body.encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def respond_json(self, data):
        payload = json.dumps(data).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main():
    parser = argparse.ArgumentParser(description="Browser UI for Pi <-> ESP32 UART testing.")
    parser.add_argument("--uart-port", default=DEFAULT_UART_PORT, help=f"UART device. Default: {DEFAULT_UART_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"UART baud rate. Default: {DEFAULT_BAUD}")
    parser.add_argument("--host", default=DEFAULT_HTTP_HOST, help=f"HTTP bind host. Default: {DEFAULT_HTTP_HOST}")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help=f"HTTP bind port. Default: {DEFAULT_HTTP_PORT}")
    args = parser.parse_args()

    bridge = SerialBridge(args.uart_port, args.baud)
    bridge.connect()

    UartRequestHandler.bridge = bridge
    server = ThreadingHTTPServer((args.host, args.http_port), UartRequestHandler)

    print(f"Open http://127.0.0.1:{args.http_port} on the Pi or http://<pi-ip>:{args.http_port} from another device.")
    print(f"UART source: {args.uart_port} @ {args.baud}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()
        bridge.disconnect()


if __name__ == "__main__":
    main()
