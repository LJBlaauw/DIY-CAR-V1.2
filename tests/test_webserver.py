# test_webserver.py — MINIMALE websocket proof-of-concept (Pico 2 W)
#
# Doel: de onzekere aannames op echte hardware testen VOORDAT we de core-split
# en de besturing eraan koppelen:
#   - komt WiFi (CYW43) op in AP-modus?
#   - draait microdot + asyncio + websocket stabiel op core 0?
#   - kan de browser tegelijk telemetrie ontvangen (push) en commando's sturen?
#
# Nog NIET aanwezig (bewust): koppeling met stepper/servo/LDR, core 1, deadman.
# Die komen pas nadat dit fundament stabiel blijkt.
#
# Vooraf eenmalig microdot installeren op de Pico 2 W (via mip, WiFi nodig, of mpremote):
#   import mip; mip.install('microdot'); mip.install('microdot.websocket')
#
# Draaien:  import test_webserver  (of via mpremote run)
# Verbinden: WiFi-netwerk "DIYCAR" -> browser naar http://192.168.4.1

import network
import json
import asyncio

from microdot import Microdot
from microdot.websocket import with_websocket

# -----------------------------------------
# WiFi — Access Point (kar als eigen netwerk, geen router nodig)
# -----------------------------------------
AP_SSID     = "DIYCAR"
AP_PASSWORD = "diycar12345"   # WPA2 vereist >= 8 tekens

def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ap.active(True)
    while not ap.active():
        pass
    print("AP actief:", AP_SSID, "->", ap.ifconfig()[0])
    return ap

# -----------------------------------------
# Dummy-telemetrie — zelfde veldnamen als de echte sensoren straks,
# zodat de browser-UI nu al klopt. Oscilleert zodat je beweging ziet.
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
        "servo":   [0, 0, 0],
        "speed":   0,
        "tick":    _t,
    }

# -----------------------------------------
# Commando-afhandeling (nu alleen loggen; later -> gedeelde state naar besturing)
# -----------------------------------------
def handle_command(msg):
    try:
        cmd = json.loads(msg)
    except Exception:
        print("ongeldig commando:", msg)
        return
    print("commando ontvangen:", cmd)

# -----------------------------------------
# Web-app
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
<div>Status: <span id="st">verbinden...</span></div>
<div>
 <div><button onclick="cmd('f')">vooruit</button></div>
 <div><button onclick="cmd('l')">links</button>
      <button onclick="cmd('stop')">STOP</button>
      <button onclick="cmd('r')">rechts</button></div>
 <div><button onclick="cmd('b')">achteruit</button></div>
</div>
<pre id="tel">wachten op data...</pre>
<script>
 let ws;
 function connect(){
   ws = new WebSocket("ws://" + location.host + "/ws");
   ws.onopen  = () => document.getElementById('st').textContent = "verbonden";
   ws.onclose = () => { document.getElementById('st').textContent = "verbroken"; setTimeout(connect, 1000); };
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
    print("websocket verbonden")
    async def sender():
        while True:
            await ws.send(json.dumps(read_telemetry()))
            await asyncio.sleep(0.2)          # ~5 Hz telemetrie
    async def receiver():
        while True:
            msg = await ws.receive()
            handle_command(msg)
    try:
        # beide richtingen tegelijk; valt weg zodra de verbinding sluit
        await asyncio.gather(sender(), receiver())
    except Exception as e:
        print("websocket gesloten:", e)

# -----------------------------------------
# Start
# -----------------------------------------
def run():
    start_ap()
    print("webserver start op poort 80 ...")
    app.run(port=80)

run()
