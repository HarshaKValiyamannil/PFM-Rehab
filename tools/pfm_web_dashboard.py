import argparse
import json
import queue
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
PREDICTION_RE = re.compile(r"^\s+(Correct|Push|Rest):\s+([0-9.]+)")
TOP_RE = re.compile(r"^Top:\s+(\w+)\s+\(([0-9.]+)\)")
ANOMALY_RE = re.compile(r"^Anomaly score:\s+([0-9.]+)")
LEDS_RE = re.compile(r"^LEDs - Green:(ON|OFF) Red:(ON|OFF) Blue:(ON|OFF)")
SAMPLE_RE = re.compile(r"^Sample\s+\d+:\s+(.+)")
WINDOW_RE = re.compile(r"^Window end:\s+(\d+)\s+ms")


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PFM Rehab Monitor</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Segoe UI, system-ui, sans-serif;
      --green: #2e7d32;
      --red: #c62828;
      --blue: #1565c0;
      --ink: #1f2933;
      --muted: #6b7280;
      --line: #d7dde5;
      --bg: #f5f7fa;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--ink);
    }
    header {
      padding: 16px 20px;
      background: white;
      border-bottom: 1px solid var(--line);
      display: flex;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
    }
    h1 {
      font-size: 18px;
      margin: 0 16px 0 0;
      font-weight: 700;
    }
    select, button {
      height: 36px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: white;
      padding: 0 10px;
      font: inherit;
    }
    button {
      cursor: pointer;
      background: #111827;
      color: white;
      border-color: #111827;
    }
    main {
      max-width: 1100px;
      margin: 0 auto;
      padding: 18px;
      display: grid;
      gap: 16px;
    }
    .grid {
      display: grid;
      grid-template-columns: minmax(260px, 1fr) minmax(280px, 1fr);
      gap: 16px;
    }
    .panel {
      background: white;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }
    .prediction {
      min-height: 180px;
      display: grid;
      place-items: center;
      text-align: center;
      background: #616161;
      color: white;
    }
    .prediction strong {
      display: block;
      font-size: clamp(42px, 9vw, 82px);
      line-height: 1;
    }
    .prediction span {
      display: block;
      margin-top: 10px;
      font-size: 18px;
    }
    .bars {
      display: grid;
      gap: 14px;
    }
    .bar-row {
      display: grid;
      grid-template-columns: 72px 1fr 58px;
      gap: 10px;
      align-items: center;
    }
    .track {
      height: 18px;
      background: #edf1f5;
      border-radius: 4px;
      overflow: hidden;
      border: 1px solid #dbe2ea;
    }
    .fill {
      height: 100%;
      width: 0%;
      background: #4b5563;
      transition: width 120ms ease-out;
    }
    .fill.Correct { background: var(--green); }
    .fill.Push { background: var(--red); }
    .fill.Rest { background: var(--blue); }
    .facts {
      display: grid;
      gap: 8px;
      color: var(--muted);
      font-size: 15px;
    }
    .facts b { color: var(--ink); }
    .samples, .log {
      height: 240px;
      overflow: auto;
      background: #0f172a;
      color: #dbeafe;
      border-radius: 6px;
      padding: 12px;
      font-family: Consolas, monospace;
      font-size: 13px;
      white-space: pre-wrap;
    }
    .samples {
      background: #172033;
      color: #e5e7eb;
    }
    @media (max-width: 760px) {
      .grid { grid-template-columns: 1fr; }
      header { align-items: stretch; }
      select, button { width: 100%; }
    }
  </style>
</head>
<body>
  <header>
    <h1>PFM Rehab Monitor</h1>
    <select id="ports"></select>
    <button id="refresh">Refresh Ports</button>
    <button id="connect">Connect</button>
    <span id="status">Not connected</span>
  </header>
  <main>
    <section class="grid">
      <div id="prediction" class="panel prediction">
        <div>
          <strong id="top">Waiting</strong>
          <span id="confidence">No prediction yet</span>
        </div>
      </div>
      <div class="panel">
        <div class="bars" id="bars"></div>
      </div>
    </section>
    <section class="grid">
      <div class="panel facts">
        <div><b>Anomaly:</b> <span id="anomaly">--</span></div>
        <div><b>LEDs:</b> <span id="leds">--</span></div>
        <div><b>Window:</b> <span id="window">--</span></div>
        <div><b>Last Update:</b> <span id="age">never</span></div>
      </div>
      <div class="panel">
        <b>Latest Samples</b>
        <div class="samples" id="samples"></div>
      </div>
    </section>
    <section class="panel">
      <b>Serial Log</b>
      <div class="log" id="log"></div>
    </section>
  </main>
  <script>
    const labels = ["Correct", "Push", "Rest"];
    const colors = { Correct: "#2e7d32", Push: "#c62828", Rest: "#1565c0", Unknown: "#616161" };
    const bars = document.getElementById("bars");
    const state = { predictions: { Correct: 0, Push: 0, Rest: 0 }, lastUpdate: null, source: null };

    labels.forEach(label => {
      const row = document.createElement("div");
      row.className = "bar-row";
      row.innerHTML = `<div>${label}</div><div class="track"><div id="fill-${label}" class="fill ${label}"></div></div><div id="value-${label}">0.000</div>`;
      bars.appendChild(row);
    });

    async function refreshPorts() {
      const response = await fetch("/ports");
      const ports = await response.json();
      const select = document.getElementById("ports");
      select.innerHTML = "";
      ports.forEach(port => {
        const option = document.createElement("option");
        option.value = port.device;
        option.textContent = `${port.device} ${port.description ? "- " + port.description : ""}`;
        select.appendChild(option);
      });
      if (!ports.length) {
        const option = document.createElement("option");
        option.textContent = "No ports found";
        select.appendChild(option);
      }
    }

    function connect() {
      const port = document.getElementById("ports").value;
      if (!port || port === "No ports found") return;
      if (state.source) state.source.close();
      state.source = new EventSource(`/events?port=${encodeURIComponent(port)}`);
      document.getElementById("status").textContent = `Connecting to ${port}...`;
      state.source.onmessage = event => handleEvent(JSON.parse(event.data));
      state.source.onerror = () => {
        document.getElementById("status").textContent = "Connection ended";
      };
    }

    function handleEvent(event) {
      if (event.type === "status") {
        document.getElementById("status").textContent = event.message;
        return;
      }
      if (event.type !== "line") return;

      appendLog(event.line);
      if (event.kind === "prediction") {
        state.predictions[event.label] = event.value;
        updateBars();
      } else if (event.kind === "top") {
        state.lastUpdate = Date.now();
        document.getElementById("top").textContent = event.label;
        document.getElementById("confidence").textContent = `Confidence ${event.value.toFixed(3)}`;
        document.getElementById("prediction").style.background = colors[event.label] || colors.Unknown;
      } else if (event.kind === "anomaly") {
        document.getElementById("anomaly").textContent = event.value.toFixed(3);
      } else if (event.kind === "leds") {
        document.getElementById("leds").textContent = `Green ${event.green} | Red ${event.red} | Blue ${event.blue}`;
      } else if (event.kind === "sample") {
        prependSample(event.value);
      } else if (event.kind === "window") {
        document.getElementById("window").textContent = `${event.value} ms`;
      }
    }

    function updateBars() {
      labels.forEach(label => {
        const value = state.predictions[label];
        document.getElementById(`fill-${label}`).style.width = `${Math.round(value * 100)}%`;
        document.getElementById(`value-${label}`).textContent = value.toFixed(3);
      });
    }

    function appendLog(line) {
      const log = document.getElementById("log");
      log.textContent += line + "\n";
      const lines = log.textContent.split("\n");
      if (lines.length > 260) log.textContent = lines.slice(-220).join("\n");
      log.scrollTop = log.scrollHeight;
    }

    function prependSample(value) {
      const samples = document.getElementById("samples");
      const lines = samples.textContent ? samples.textContent.split("\n") : [];
      lines.unshift(value);
      samples.textContent = lines.slice(0, 12).join("\n");
    }

    setInterval(() => {
      const age = document.getElementById("age");
      if (!state.lastUpdate) {
        age.textContent = "never";
      } else {
        age.textContent = `${((Date.now() - state.lastUpdate) / 1000).toFixed(1)}s ago`;
      }
    }, 250);

    document.getElementById("refresh").addEventListener("click", refreshPorts);
    document.getElementById("connect").addEventListener("click", connect);
    refreshPorts();
  </script>
</body>
</html>
"""


def parse_line(line):
    prediction = PREDICTION_RE.match(line)
    if prediction:
        label, value = prediction.groups()
        return {"kind": "prediction", "label": label, "value": float(value), "line": line}

    top = TOP_RE.match(line)
    if top:
        label, value = top.groups()
        return {"kind": "top", "label": label, "value": float(value), "line": line}

    anomaly = ANOMALY_RE.match(line)
    if anomaly:
        return {"kind": "anomaly", "value": float(anomaly.group(1)), "line": line}

    leds = LEDS_RE.match(line)
    if leds:
        green, red, blue = leds.groups()
        return {"kind": "leds", "green": green, "red": red, "blue": blue, "line": line}

    sample = SAMPLE_RE.match(line)
    if sample:
        return {"kind": "sample", "value": sample.group(1), "line": line}

    window = WINDOW_RE.match(line)
    if window:
        return {"kind": "window", "value": int(window.group(1)), "line": line}

    return {"kind": "raw", "line": line}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML.encode("utf-8"))
            return

        if parsed.path == "/ports":
            ports = [
                {"device": port.device, "description": port.description}
                for port in list_ports.comports()
            ]
            self.send_json(ports)
            return

        if parsed.path == "/events":
            params = parse_qs(parsed.query)
            port = params.get("port", [""])[0]
            if not port:
                self.send_error(400, "Missing port")
                return
            self.stream_events(port)
            return

        self.send_error(404)

    def send_json(self, payload):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def stream_events(self, port):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        def send(payload):
            data = f"data: {json.dumps(payload)}\n\n".encode("utf-8")
            self.wfile.write(data)
            self.wfile.flush()

        try:
            with serial.Serial(port, BAUD_RATE, timeout=0.2) as handle:
                send({"type": "status", "message": f"Connected to {port} at {BAUD_RATE}"})
                while True:
                    raw = handle.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        payload = parse_line(line)
                        payload["type"] = "line"
                        send(payload)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            try:
                send({"type": "status", "message": f"Serial error: {exc}"})
            except Exception:
                pass

    def log_message(self, format, *args):
        return


def main():
    parser = argparse.ArgumentParser(description="PFM Rehab serial dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"PFM dashboard running at http://{args.host}:{args.port}")
    print("Open that URL in your browser, choose the Arduino COM port, then Connect.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
