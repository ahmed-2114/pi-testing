#!/usr/bin/env python3
"""Headless web dashboard for Audix control and monitoring."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import rclpy
from audix_interfaces.msg import EspTelemetry, IrState
from audix_interfaces.srv import (
    AuditMission,
    DirectionCommand,
    LiftMoveSteps,
    RotateCommand,
    SetRobotMode,
    ShelfScan,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


INDEX_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Audix Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0d10;
      --panel: #151b22;
      --line: #2e3a46;
      --text: #f2f5f8;
      --muted: #9aa8b4;
      --accent: #38c6a3;
      --warn: #efb75c;
      --danger: #e05757;
      --blue: #73a9ff;
    }
    * { box-sizing: border-box; letter-spacing: 0; }
    body { margin: 0; background: var(--bg); color: var(--text); font-family: Segoe UI, Arial, sans-serif; }
    main { max-width: 1220px; margin: 0 auto; padding: 18px; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 14px; }
    h1 { margin: 0; font-size: 26px; font-weight: 650; }
    h2 { margin: 0 0 10px; font-size: 16px; font-weight: 650; }
    .grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 12px; }
    section { grid-column: span 6; border-top: 1px solid var(--line); padding-top: 12px; }
    .wide { grid-column: span 12; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px; }
    .metrics { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 8px; }
    .metric { border: 1px solid var(--line); border-radius: 6px; padding: 8px; min-height: 56px; }
    .metric span { display: block; color: var(--muted); font-size: 12px; }
    .metric strong { display: block; font-size: 20px; margin-top: 4px; overflow-wrap: anywhere; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
    button, input, select {
      border: 1px solid var(--line); border-radius: 6px; background: #11171e; color: var(--text);
      padding: 9px 10px; font: inherit;
    }
    button { cursor: pointer; min-width: 72px; }
    button:hover { border-color: var(--accent); }
    .danger { background: var(--danger); border-color: var(--danger); color: white; font-weight: 700; }
    .accent { background: #123b34; border-color: var(--accent); }
    .warn { background: #3f301b; border-color: var(--warn); }
    input[type=number] { width: 110px; }
    .pad { display: grid; grid-template-columns: repeat(3, 88px); grid-auto-rows: 48px; gap: 8px; }
    .pad button { min-width: 0; }
    .log { min-height: 110px; max-height: 220px; overflow: auto; color: var(--muted); font-family: Consolas, monospace; white-space: pre-wrap; }
    .ir { display: grid; grid-template-columns: repeat(6, minmax(0,1fr)); gap: 8px; }
    .pill { border: 1px solid var(--line); border-radius: 6px; padding: 8px; text-align: center; color: var(--muted); }
    .pill.on { color: white; background: #4a1c22; border-color: var(--danger); }
    .sides { display: grid; grid-template-columns: repeat(2, minmax(0,1fr)); gap: 8px; }
    .vision { display: grid; grid-template-columns: repeat(3, minmax(0,1fr)); gap: 8px; margin-top: 10px; }
    .camera-feed { width: 100%; max-height: 320px; object-fit: contain; border: 1px solid var(--line); border-radius: 6px; background: #07090c; margin-bottom: 10px; }
    .scan-image { display: none; width: 100%; max-height: 260px; object-fit: contain; border: 1px solid var(--line); border-radius: 6px; margin-top: 10px; background: #07090c; }
    label { color: var(--muted); font-size: 13px; }
    @media (max-width: 860px) { section { grid-column: span 12; } .metrics { grid-template-columns: repeat(2, minmax(0,1fr)); } .sides, .vision { grid-template-columns: repeat(1, minmax(0,1fr)); } }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Audix Control</h1>
      <div id="event" style="color:var(--muted)">connecting...</div>
    </div>
    <button class="danger" onclick="stopRobot()">STOP</button>
  </header>

  <div class="grid">
    <section class="wide">
      <div class="metrics">
        <div class="metric"><span>Mode</span><strong id="mode">-</strong></div>
        <div class="metric"><span>Telemetry age</span><strong id="age">-</strong></div>
        <div class="metric"><span>Forward cm</span><strong id="forward">0</strong></div>
        <div class="metric"><span>Strafe cm</span><strong id="strafe">0</strong></div>
        <div class="metric"><span>Yaw deg</span><strong id="yaw">0</strong></div>
        <div class="metric"><span>IMU</span><strong id="imu">-</strong></div>
        <div class="metric"><span>Move</span><strong id="move">-</strong></div>
        <div class="metric"><span>Last seq</span><strong id="seq">-</strong></div>
      </div>
    </section>

    <section>
      <h2>Manual Jog</h2>
      <div class="panel">
        <div class="row">
          <label>Distance cm <input id="dist" type="number" min="1" max="300" step="1" value="20" /></label>
          <button onclick="setMode('manual')">Manual</button>
          <button onclick="setMode('mission')">Mission</button>
        </div>
        <div class="pad">
          <button onclick="moveDir('FL')">FL</button>
          <button onclick="moveDir('F')">F</button>
          <button onclick="moveDir('FR')">FR</button>
          <button onclick="moveDir('L')">L</button>
          <button class="danger" onclick="stopRobot()">STOP</button>
          <button onclick="moveDir('R')">R</button>
          <button onclick="moveDir('BL')">BL</button>
          <button onclick="moveDir('B')">B</button>
          <button onclick="moveDir('BR')">BR</button>
        </div>
        <div class="row" style="margin-top:10px">
          <label>Rotate deg <input id="rotdeg" type="number" min="1" max="360" step="5" value="90" /></label>
          <button onclick="rotate('left')">Rotate Left</button>
          <button onclick="rotate('right')">Rotate Right</button>
        </div>
        <div class="row">
          <button onclick="trigger('/api/init_imu')">Init IMU</button>
          <button onclick="trigger('/api/reset_odom')">Reset Odom</button>
          <button onclick="homeRobot()">Home</button>
          <button onclick="buzzer(true)">Buzzer On</button>
          <button onclick="buzzer(false)">Buzzer Off</button>
        </div>
      </div>
    </section>

    <section>
      <h2>IR Sensors</h2>
      <div class="panel ir">
        <div id="ir_front_left" class="pill">FL</div>
        <div id="ir_front" class="pill">Front</div>
        <div id="ir_front_right" class="pill">FR</div>
        <div id="ir_left" class="pill">Left</div>
        <div id="ir_right" class="pill">Right</div>
        <div id="ir_back" class="pill">Back</div>
      </div>
    </section>

    <section class="wide">
      <h2>Mission Audit</h2>
      <div class="panel">
        <div class="sides">
          <label><input type="checkbox" class="side" value="1" checked /> Lane 1 side</label>
          <label><input type="checkbox" class="side" value="2" /> Lane 2 side</label>
        </div>
        <div class="row" style="margin-top:10px">
          <label><input id="level1" type="checkbox" checked /> Level 1</label>
          <label><input id="level2" type="checkbox" checked /> Level 2</label>
          <button class="accent" onclick="startAudit()">Start Audit</button>
          <button class="warn" onclick="lift(500)">Jog +500</button>
          <button class="warn" onclick="lift(-500)">Jog -500</button>
        </div>
      </div>
    </section>

    <section class="wide">
      <h2>Vision</h2>
      <div class="panel">
        <img id="cameraFeed" class="camera-feed" alt="camera feed" />
        <div class="row">
          <select id="scanShelf">
            <option value="indomie">Indomie</option>
            <option value="beans_can">Beans Can</option>
            <option value="fruit_rings_cereal">Fruit Rings</option>
          </select>
          <button class="accent" onclick="scanShelf()">Scan</button>
        </div>
        <div class="vision">
          <div class="metric"><span>Status</span><strong id="scanStatus">-</strong></div>
          <div class="metric"><span>Count</span><strong id="scanCount">-</strong></div>
          <div class="metric"><span>Confidence</span><strong id="scanConfidence">-</strong></div>
        </div>
        <div id="scanMessage" style="color:var(--muted); margin-top:8px; overflow-wrap:anywhere"></div>
        <img id="scanImage" class="scan-image" alt="latest scan" />
      </div>
    </section>

    <section class="wide">
      <h2>Log</h2>
      <div id="log" class="panel log"></div>
    </section>
  </div>
</main>
<script>
const logEl = document.getElementById('log');
let lastSeenEvent = '';
let lastCameraOk = false;
function log(msg) {
  const t = new Date().toLocaleTimeString();
  logEl.textContent = `[${t}] ${msg}\n` + logEl.textContent;
}
async function post(path, body={}) {
  const res = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await res.json();
  log(`${path}: ${JSON.stringify(data)}`);
  return data;
}
function dist() { return Number(document.getElementById('dist').value || 0); }
function rotdeg() { return Number(document.getElementById('rotdeg').value || 0); }
function moveDir(direction) { post('/api/move', {direction, distance_cm: dist()}); }
function rotate(direction) { post('/api/rotate', {direction, degrees: rotdeg()}); }
function stopRobot() { post('/api/stop'); }
function homeRobot() { post('/api/home'); }
function setMode(mode) { post('/api/mode', {mode}); }
function trigger(path) { post(path); }
function buzzer(on) { post('/api/buzzer', {on}); }
function lift(steps) { post('/api/lift', {steps}); }
async function scanShelf() {
  const shelf_id = document.getElementById('scanShelf').value;
  const data = await post('/api/scan', {shelf_id});
  if (data.scan) renderScan(data.scan);
}
function startAudit() {
  const shelves = [...document.querySelectorAll('.side:checked')].map(x => Number(x.value));
  post('/api/audit', {shelves, level_1: document.getElementById('level1').checked, level_2: document.getElementById('level2').checked});
}
function setText(id, value) { document.getElementById(id).textContent = value; }
function setIr(id, active) { document.getElementById(id).classList.toggle('on', !!active); }
function renderScan(scan) {
  setText('scanStatus', scan?.status || '-');
  const count = scan ? `${scan.detected_count ?? 0}/${scan.expected_count ?? 0}` : '-';
  setText('scanCount', count);
  const confidence = Number(scan?.confidence ?? 0);
  setText('scanConfidence', scan ? confidence.toFixed(2) : '-');
  const wrong = (scan?.wrong_products || []).join(', ');
  const location = scan?.side && scan?.level ? `side ${scan.side} level ${scan.level} | ` : '';
  document.getElementById('scanMessage').textContent = scan ? `${location}${scan.shelf_id}: ${scan.message || ''}${wrong ? ' | wrong: ' + wrong : ''}` : '';
  const img = document.getElementById('scanImage');
  if (scan?.image_path) {
    img.src = `/api/audit_image?path=${encodeURIComponent(scan.image_path)}&t=${Date.now()}`;
    img.style.display = 'block';
  } else {
    img.removeAttribute('src');
    img.style.display = 'none';
  }
}
function refreshCamera() {
  const img = document.getElementById('cameraFeed');
  img.src = `/api/camera.jpg?t=${Date.now()}`;
}
async function refresh() {
  try {
    const res = await fetch('/api/status');
    const s = await res.json();
    setText('mode', s.mode || '-');
    setText('event', s.last_event || '');
    if (s.last_event && s.last_event !== lastSeenEvent) {
      lastSeenEvent = s.last_event;
      log(`event: ${s.last_event}`);
    }
    setText('age', Number(s.telemetry_age_s ?? 0).toFixed(2) + 's');
    setText('forward', Number(s.telemetry?.forward_cm ?? 0).toFixed(1));
    setText('strafe', Number(s.telemetry?.strafe_cm ?? 0).toFixed(1));
    setText('yaw', Number(s.telemetry?.yaw_deg ?? 0).toFixed(1));
    setText('imu', s.telemetry?.imu_ok ? 'OK' : 'check');
    setText('move', s.telemetry?.mode || '-');
    setText('seq', s.telemetry?.seq ?? '-');
    for (const [name, active] of Object.entries(s.ir || {})) setIr('ir_' + name, active);
    if (s.latest_scan) renderScan(s.latest_scan);
  } catch (e) {
    setText('event', 'dashboard disconnected');
  }
}
setInterval(refresh, 250);
setInterval(refreshCamera, 300);
refresh();
refreshCamera();
</script>
</body>
</html>
"""


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("web_dashboard")
        self.callback_group = ReentrantCallbackGroup()
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8080).value)
        self.mode = "manual"
        self.last_event = "ready"
        self.latest_ir: dict[str, bool] = {}
        self.latest_telemetry: dict[str, Any] = {}
        self.latest_scan: dict[str, Any] | None = None
        self.telemetry_stamp = 0.0
        self.audit_image_dir = str(self.declare_parameter("audit_image_dir", "~/audix/audit_images").value)
        self.camera_jpeg_quality = int(self.declare_parameter("camera_jpeg_quality", 75).value)
        self.camera_min_period_s = float(self.declare_parameter("camera_min_period_s", 0.20).value)
        self.camera_lock = threading.Lock()
        self.camera_bridge = CvBridge()
        self.latest_camera_jpeg: bytes | None = None
        self.camera_stamp = 0.0
        self.last_camera_encode_s = 0.0

        self.direction_client = self.create_client(DirectionCommand, "manager/direction_move", callback_group=self.callback_group)
        self.rotate_client = self.create_client(RotateCommand, "manager/rotate", callback_group=self.callback_group)
        self.mode_client = self.create_client(SetRobotMode, "manager/set_mode", callback_group=self.callback_group)
        self.audit_client = self.create_client(AuditMission, "manager/start_audit", callback_group=self.callback_group)
        self.home_client = self.create_client(Trigger, "manager/go_home", callback_group=self.callback_group)
        self.stop_client = self.create_client(Trigger, "manager/stop", callback_group=self.callback_group)
        self.init_imu_client = self.create_client(Trigger, "esp/init_imu", callback_group=self.callback_group)
        self.reset_odom_client = self.create_client(Trigger, "esp/reset_odom", callback_group=self.callback_group)
        self.buzzer_client = self.create_client(SetBool, "gpio/set_buzzer", callback_group=self.callback_group)
        self.lift_client = self.create_client(LiftMoveSteps, "lift/move_steps", callback_group=self.callback_group)
        self.scan_client = self.create_client(ShelfScan, "scan_shelf", callback_group=self.callback_group)

        self.create_subscription(IrState, "ir/state", self._on_ir, 10, callback_group=self.callback_group)
        self.create_subscription(EspTelemetry, "esp/telemetry", self._on_telemetry, 10, callback_group=self.callback_group)
        self.create_subscription(Image, "image_raw", self._on_camera, 5, callback_group=self.callback_group)
        self.create_subscription(String, "mission/event", self._on_event, 20, callback_group=self.callback_group)
        self.create_subscription(String, "vision/scan_result", self._on_scan_result, 10, callback_group=self.callback_group)

        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"Web dashboard ready at http://{self.host}:{self.port}")

    def _on_ir(self, msg: IrState) -> None:
        self.latest_ir = {
            "front_left": bool(msg.front_left),
            "front": bool(msg.front),
            "front_right": bool(msg.front_right),
            "left": bool(msg.left),
            "right": bool(msg.right),
            "back": bool(msg.back),
        }

    def _on_telemetry(self, msg: EspTelemetry) -> None:
        self.telemetry_stamp = self.get_clock().now().nanoseconds / 1e9
        self.latest_telemetry = {
            "mode": msg.mode,
            "seq": int(msg.seq),
            "imu_ok": bool(msg.imu_ok),
            "yaw_deg": float(msg.yaw_deg),
            "forward_cm": float(msg.forward_cm),
            "strafe_cm": float(msg.strafe_cm),
            "progress_cm": float(msg.progress_cm),
            "remaining_cm": float(msg.remaining_cm),
            "move_done": bool(msg.move_done),
        }

    def _on_event(self, msg: String) -> None:
        self.last_event = msg.data

    def _on_scan_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.latest_scan = {
                "success": bool(data.get("success", False)),
                "side": int(data.get("side", 0)),
                "level": int(data.get("level", 0)),
                "shelf_id": str(data.get("shelf_id", "")),
                "expected_product": str(data.get("expected_product", "")),
                "expected_count": int(data.get("expected_count", 0)),
                "detected_count": int(data.get("detected_count", 0)),
                "detected_products": list(data.get("detected_products", [])),
                "wrong_products": list(data.get("wrong_products", [])),
                "confidence": float(data.get("confidence", 0.0)),
                "status": str(data.get("status", "")),
                "message": str(data.get("message", "")),
                "image_path": str(data.get("image_path", "")),
            }
        except Exception as exc:
            self.get_logger().warning(f"bad scan result message: {exc}")

    def _on_camera(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_camera_encode_s < max(0.05, self.camera_min_period_s):
            return
        try:
            frame = self.camera_bridge.imgmsg_to_cv2(msg, "bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), max(25, min(95, self.camera_jpeg_quality))],
            )
            if not ok:
                return
        except Exception as exc:
            self.get_logger().warning(f"camera encode failed: {exc}")
            return
        with self.camera_lock:
            self.latest_camera_jpeg = encoded.tobytes()
            self.camera_stamp = now
            self.last_camera_encode_s = now

    def _call_sync(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=max(0.1, float(timeout_s))):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        event = threading.Event()
        holder = {}
        future = client.call_async(request)
        future.add_done_callback(lambda done: (holder.setdefault("future", done), event.set()))
        if not event.wait(timeout_s):
            raise TimeoutError(f"timed out waiting for {client.srv_name}")
        return holder["future"].result()

    def _status(self) -> dict[str, Any]:
        now = self.get_clock().now().nanoseconds / 1e9
        return {
            "mode": self.mode,
            "last_event": self.last_event,
            "ir": self.latest_ir,
            "telemetry": self.latest_telemetry,
            "telemetry_age_s": max(0.0, now - self.telemetry_stamp) if self.telemetry_stamp else None,
            "latest_scan": self.latest_scan,
            "camera_age_s": max(0.0, now - self.camera_stamp) if self.camera_stamp else None,
        }

    @staticmethod
    def _scan_to_dict(res) -> dict[str, Any]:
        return {
            "success": bool(res.success),
            "shelf_id": str(res.shelf_id),
            "expected_product": str(res.expected_product),
            "expected_count": int(res.expected_count),
            "detected_count": int(res.detected_count),
            "detected_products": list(res.detected_products),
            "wrong_products": list(res.wrong_products),
            "confidence": float(res.confidence),
            "status": str(res.status),
            "message": str(res.message),
            "image_path": str(res.image_path),
        }

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format: str, *_args) -> None:
                return

            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _json(self, payload: dict[str, Any], status: int = 200) -> None:
                self._send(status, json.dumps(payload).encode("utf-8"), "application/json")

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0") or "0")
                if length <= 0:
                    return {}
                return json.loads(self.rfile.read(length).decode("utf-8"))

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == "/" or parsed.path == "/index.html":
                    self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif parsed.path == "/api/status":
                    self._json(node._status())
                elif parsed.path == "/api/audit_image":
                    params = parse_qs(parsed.query)
                    image_path = params.get("path", [""])[0]
                    self._send_audit_image(image_path)
                elif parsed.path == "/api/camera.jpg":
                    self._send_camera_frame()
                else:
                    self._json({"ok": False, "message": "not found"}, 404)

            def _send_camera_frame(self) -> None:
                with node.camera_lock:
                    frame = node.latest_camera_jpeg
                if frame is None:
                    self._json({"ok": False, "message": "no camera frame yet"}, 404)
                    return
                self._send(200, frame, "image/jpeg")

            def _send_audit_image(self, image_path: str) -> None:
                base = Path(node.audit_image_dir).expanduser().resolve()
                try:
                    requested = Path(image_path).expanduser().resolve()
                    if not requested.is_file() or (requested != base and base not in requested.parents):
                        self._json({"ok": False, "message": "image not found"}, 404)
                        return
                    self._send(200, requested.read_bytes(), "image/jpeg")
                except Exception as exc:
                    self._json({"ok": False, "message": str(exc)}, 404)

            def do_POST(self) -> None:
                try:
                    data = self._read_json()
                    if self.path == "/api/move":
                        req = DirectionCommand.Request()
                        req.direction = str(data.get("direction", ""))
                        req.distance_cm = float(data.get("distance_cm", 0.0))
                        req.timeout_s = float(data.get("timeout_s", 0.0))
                        res = node._call_sync(node.direction_client, req, 200.0)
                        self._json({"ok": res.ok, "result": res.result, "message": res.message})
                    elif self.path == "/api/rotate":
                        req = RotateCommand.Request()
                        req.direction = str(data.get("direction", ""))
                        req.degrees = float(data.get("degrees", 0.0))
                        req.timeout_s = float(data.get("timeout_s", 10.0))
                        res = node._call_sync(node.rotate_client, req, 30.0)
                        self._json({"ok": res.ok, "result": res.result, "message": res.message, "heading_deg": res.heading_deg})
                    elif self.path == "/api/mode":
                        req = SetRobotMode.Request()
                        req.mode = str(data.get("mode", "manual"))
                        res = node._call_sync(node.mode_client, req, 5.0)
                        if res.ok:
                            node.mode = res.active_mode
                        self._json({"ok": res.ok, "message": res.message, "mode": res.active_mode})
                    elif self.path == "/api/audit":
                        req = AuditMission.Request()
                        req.shelves = [int(v) for v in data.get("shelves", [])]
                        req.level_1 = bool(data.get("level_1", True))
                        req.level_2 = bool(data.get("level_2", True))
                        res = node._call_sync(node.audit_client, req, 5.0)
                        self._json({"ok": res.accepted, "message": res.message})
                    elif self.path == "/api/stop":
                        res = node._call_sync(node.stop_client, Trigger.Request(), 5.0)
                        node.mode = "manual"
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/home":
                        res = node._call_sync(node.home_client, Trigger.Request(), 220.0)
                        node.mode = "manual"
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/init_imu":
                        res = node._call_sync(node.init_imu_client, Trigger.Request(), 12.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/reset_odom":
                        res = node._call_sync(node.reset_odom_client, Trigger.Request(), 5.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/buzzer":
                        req = SetBool.Request()
                        req.data = bool(data.get("on", False))
                        res = node._call_sync(node.buzzer_client, req, 3.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/lift":
                        raw_steps = int(data.get("steps", 0))
                        if "direction" in data:
                            direction = 1 if int(data.get("direction", 1)) >= 0 else -1
                            signed_steps = abs(raw_steps) * direction
                        else:
                            signed_steps = raw_steps
                        req = LiftMoveSteps.Request()
                        req.steps = abs(signed_steps)
                        req.direction = 1 if signed_steps >= 0 else -1
                        req.speed_sps = float(data.get("speed_sps", 500.0))
                        res = node._call_sync(node.lift_client, req, 15.0)
                        self._json({"ok": res.ok, "message": res.message})
                    elif self.path == "/api/scan":
                        req = ShelfScan.Request()
                        req.shelf_id = str(data.get("shelf_id", ""))
                        res = node._call_sync(node.scan_client, req, 30.0)
                        scan = node._scan_to_dict(res)
                        node.latest_scan = scan
                        self._json({"ok": res.success, "message": res.message, "scan": scan})
                    else:
                        self._json({"ok": False, "message": "not found"}, 404)
                except Exception as exc:
                    self._json({"ok": False, "message": str(exc)}, 500)

        return Handler

    def destroy_node(self) -> bool:
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
