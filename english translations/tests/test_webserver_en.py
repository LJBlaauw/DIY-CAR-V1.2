# test_webserver.py — MINIMAL websocket proof-of-concept (Pico 2 W)
#
# Goal: test the uncertain assumptions on real hardware BEFORE we link the core-split
# and the control logic:
#   - does WiFi (CYW43) come up in AP mode?
#   - does microdot + asyncio + websocket run stably on core 0?
#   - can the browser simultaneously receive telemetry (push) and send commands?
#
# Not YET present (deliberately): link with stepper/servo/LDR, core 1, deadman.
# These will only be added after this foundation proves stable.
#
# One-time microdot installation beforehand on the Pico 2 W (via mip, WiFi required, or mpremote):
#   import mip; mip.install('microdot'); mip.install('microdot.websocket')
#
# Run:       import test_webserver  (or via mpremote run)
# Connect:   WiFi network "DIYCAR" -> browser to http://192.168.4.1

import network
import json
import asyncio

from microdot import Microdot
from microdot.websocket import with_websocket

# -----------------------------------------
# WiFi — Access Point (cart as its own network, no router needed)
# -----------------------------------------
AP_SSID     = "DIYCAR"
AP_PASSWORD = "diycar12345"   # WPA2 requires >= 8 characters

def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ap.active(True)
    while not ap.active():
        pass
    print("AP active:", AP_SSID, "->", ap.ifconfig()[0])
    return ap

# -----------------------------------------
# Dummy telemetry — same field names as the real sensors later,
# so the browser UI already works now. Oscillates so you see movement.
# -----------------------------------------
_t = 0
def read_telemetry():
    global _t
    _t += 1
    phase = _t % 100
    return {
        "ldr_a":   round(50 + 40 * (phase / 100.0), 1),
        "ldr_b":   round(90 - 40 * (phase / 100.0), 1),
        "dist_cm": round(20 + phase * 0.5, 1),
        "heading": (phase * 3) % 360 - 180,
        "servo":,
        "speed":   0,
        "tick":    _t,
    }

# -----------------------------------------
# Command handling (now only logging; later -> shared state to control logic)
# -----------------------------------------
def handle_command(msg):
    try:
        cmd = json.loads(msg)
    except Exception:
        print("invalid command:", msg)
        return
    print("command received:", cmd)

# -----------------------------------------
# Web app
# -----------------------------------------
app = Microdot()

PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DIY CAR</title>
<style>
 body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:1em}
 button{width:5em;height:3em;margin:.3em;font-size:1.2em;border-radius:.4em;border:0;background:#333;color:#eee}
 button:active{background:#0a6}
 #tel{text-align:left;display:inline-block;background:#000;padding:1em;border-radius:.5em;margin-top:1em;min-width:16em}
 #st{color:#0a6}
</style></head><body>
<h2>DIY CAR — POC</h2>
<div>Status: <span id="st">connecting...</span></div>
<div>
 <div><button onclick="cmd('f')">forward</button></div>
 <div><button onclick="cmd('l')">left</button>
      <button onclick="cmd('stop')">STOP</button>
      <button onclick="cmd('r')">right</button></div>
 <div><button onclick="cmd('b')">backward</button></div>
</div>
<pre id="tel">waiting for data...</pre>
<script>
 let ws;
 function connect(){
   ws = new WebSocket("ws://" + location.host + "/ws");
   ws.onopen  = () => document.getElementById('st').textContent = "connected";
   ws.onclose = () => { document.getElementById('st').textContent = "disconnected"; setTimeout(connect, 1000); };
   ws.onmessage = (e) => { document.getElementById('tel').textContent = JSON.stringify(JSON.parse(e.data), null, 2); };
 }
 function cmd(d){ if(ws && ws.readyState===1) ws.send(JSON.stringify({move:d})); }
 connect();
</script>
</body></html>"""

@app.route("/")
async def index(request):
    return PAGE, 200, {"Content-Type": "text/html"}

@app.route("/ws")
@with_websocket
async def ws_handler(request, ws):
    print("websocket connected")
    async def sender():
        while True:
            await ws.send(json.dumps(read_telemetry()))
            await asyncio.sleep(0.2)          # ~5 Hz telemetry
    async def receiver():
        while True:
            msg = await ws.receive()
            handle_command(msg)
    try:
        # both directions at once; drops out as soon as the connection closes
        await asyncio.gather(sender(), receiver())
    except Exception as e:
        print("websocket closed:", e)

# -----------------------------------------
# Start
# -----------------------------------------
def run():
    start_ap()
    print("webserver starting on port 80 ...")
    app.run(port=80)

run()
