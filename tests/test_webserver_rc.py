# ================================================================
# tests/test_webserver_rc.py
#
# Teleoperatie via de browser: twee joysticks + kompas + scan/laser/stop,
# gekoppeld aan de ECHTE sensoren en besturing (opvolger van de
# test_webserver.py-proof-of-concept, die alleen neptelemetrie en
# log-only-commando's had).
#
# UI (landscape telefoon):
#   linksboven   : afstand (ultrasoon + ToF)
#   middenboven  : LDR A/B %, stuurpijl, SCAN- en LASER-knop
#   rechtsboven  : kompas (360 graden, N/NO/O/ZO/Z/ZW/W/NW)
#   linksonder   : arm/grijper-joystick (omhoog=strekken, omlaag=intrekken,
#                  links=grijper dicht, rechts=grijper open)
#   rechtsonder  : rij-joystick (omhoog=voorwaarts, omlaag=achteruit,
#                  links=bocht links, rechts=bocht rechts)
#   onderin      : NOODSTOP-knop
#
# ARCHITECTUUR
# ------------
# - AP-modus (SSID/wachtwoord uit config.json, sectie "network").
# - EEN permanente achtergrondtaak (control_task) rijdt/stuurt de arm en
#   bewaakt de deadman -- deze blijft lopen ONGEACHT of er een browser
#   verbonden is. Zonder nieuw commando binnen DEADMAN_MS stopt de kar
#   vanzelf (zie control_task). Dit is de veiligheidsgrens, niet de
#   websocket-verbinding zelf.
# - Per websocket-verbinding: sender() stuurt telemetrie (~10 Hz) uit de
#   gedeelde _state; receiver() verwerkt joystick-berichten (-> _cmd) en
#   losse commando's (scan/laser/stop).
# - Rijden gaat via stepper_ramp.LiveDrive (arcade-mixing, software-
#   versnellingslimiet) -- GEEN vaste afstand/HeadingController zoals Move,
#   de bestuurder IS hier de regelaar.
# - Arm/grijper via ServoController.servo_pos(): J1+J2 ALTIJD gelijktijdig
#   aangestuurd (botsingsrandvoorwaarde, zie lib/servo/servo_crl.py en
#   tests/test_servo.py); de grijper (J10) onafhankelijk daarvan.
# - Veiligheidsstop tijdens het rijden: de kortste van ultrasoon/ToF wordt
#   tegen geo.STOP_DIST_CM gehouden en blokkeert alleen verder VOORUIT rijden
#   (achteruit/draaien blijft altijd mogelijk).
#
# BEKENDE BEPERKING: ldr.scan() is blokkerend (meerdere seconden). Tijdens
# een scan bevriezen telemetrie EN besturing kortstondig -- dit is bewust
# aanvaard (een scan is een stilstaande actie), maar de kar wordt daarom
# VOOR de scan expliciet stilgezet (ld.stop()) zodat er nooit onbestuurd
# doorgereden wordt tijdens die bevriezing. Een volledige async-refactor van
# scan() staat nog open (zie globale_specificatie_NL.md -> webserver-sectie).
#
# Vereist: GPIO2 -> GPIO5 fysiek verbonden (servo-PWM-tick voor
# ldr.start_synced_sampling(), zie lib/servo/servo_crl.py), net als
# tests/test_drive.py.
# ================================================================

import time
import json
import network
import asyncio
import machine

from microdot import Microdot
from microdot_websocket import with_websocket

import stepper_ramp as sr
import ldr_scan_isr as ldr
import ultrasoon
from servo_crl import ServoController
from icm20948 import ICM20948, load_mag_calibration
from vl53l1x import VL53L1X, LONG, RANGE_VALID
import geometry as geo

# ----------------------------------------------------------------
# Instellingen
# ----------------------------------------------------------------
CONFIG_PATH = "config.json"


def _load_network_config():
    """Lees de "network"-sectie uit config.json; {} als die ontbreekt of
    corrupt is (zelfde patroon als servo_crl.py/ldr_scan_isr.py/vl53l1x.py)."""
    try:
        with open(CONFIG_PATH) as f:
            return json.load(f).get("network", {})
    except Exception:
        return {}


_net_cfg = _load_network_config()
AP_SSID = _net_cfg.get("ap_ssid", "DIYCAR")
AP_PASSWORD = _net_cfg.get("ap_password", "diycar12345")
del _net_cfg

DEADMAN_MS = 400          # geen joystick-bericht binnen deze tijd -> noodstop
TELEMETRY_HZ = 10
ARM_MAX_SPEED_DEG_S = 60.0
TOF_TIMING_BUDGET_US = 33_000
TOF_PERIOD_MS = 50

# ----------------------------------------------------------------
# WiFi -- Access Point
# ----------------------------------------------------------------
def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ap.active(True)
    while not ap.active():
        pass
    print("AP actief:", AP_SSID, "->", ap.ifconfig()[0])
    return ap


# ----------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------
sc = ServoController()
sc.servo_cur_limit(200)   # stroomlimiet grijper, zelfde als test_drive.py/test_servo.py

i2c0 = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                   freq=400000, timeout=200_000)   # OLED-bus, hier: ToF
i2c1 = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)

sensor = ICM20948(i2c1, **load_mag_calibration())

tof = VL53L1X(i2c0)
_tof_ok = tof.init()
if _tof_ok:
    tof.set_distance_mode(LONG)
    tof.set_timing_budget_us(TOF_TIMING_BUDGET_US)
    tof.start_continuous(TOF_PERIOD_MS)
else:
    print("! VL53L1X.init() mislukt -- ToF-afstand blijft leeg")

ldr.start_synced_sampling(sc)
time.sleep_ms(200)   # eerste tick(s) laten landen voor read_synced()

ld = sr.LiveDrive()

# ----------------------------------------------------------------
# Gedeelde staat (control_task schrijft, sender() leest; receiver() schrijft
# _cmd/_last_cmd_ms, control_task leest). Geen lock nodig: asyncio is
# single-threaded coöperatief, en elke schrijf/lees hieronder is een simpele
# toewijzing zonder "await" ertussen.
# ----------------------------------------------------------------
_cmd = {"fwd": 0.0, "turn": 0.0, "reach": 0.0, "grip": 0.0}
_last_cmd_ms = time.ticks_ms()
_laser_on = False
_scanning = False
_state = {
    "heading": 0.0, "ldr_a": 0.0, "ldr_b": 0.0,
    "us_cm": None, "us_ok": False,
    "tof_cm": None, "tof_ok": False,
    "j1_pct": 0.0, "j2_pct": 0.0, "grip_pct": 0.0,
    "speed_cm_s": 0.0, "turn_cm_s": 0.0,
    "laser": False, "scanning": False, "deadman_ms": 0,
}


# ----------------------------------------------------------------
# Scan / laser -- eenmalige commando's (BLOKKEREND tijdens scan(), zie kop)
# ----------------------------------------------------------------
def _do_scan():
    global _scanning
    ld.stop()                 # nooit onbestuurd doorrijden tijdens de bevriezing
    _scanning = True
    _state["scanning"] = True
    try:
        res = ldr.scan('l', 5.0, go_max=True, excel=False)
        print("scan-resultaat:", res)
    finally:
        _scanning = False
        _state["scanning"] = False


def _toggle_laser():
    global _laser_on
    _laser_on = not _laser_on
    if _laser_on:
        sc.laser_power(100.0)
    else:
        sc.laser_off()
    _state["laser"] = _laser_on


# ----------------------------------------------------------------
# Arm/grijper -- niet-blokkend (servo_pos() zet alleen een doel+snelheid;
# de daadwerkelijke beweging loopt via de hard-IRQ-gekoppelde update_all(),
# zie lib/servo/servo_crl.py -- hier hoeft niets "geservice" te worden).
# ----------------------------------------------------------------
def _freeze(nr):
    """Bevries servo `nr` op zijn HUIDIGE positie (doel = huidige pos)."""
    s = sc.servos[nr]
    s["target_deg"] = s["pos_deg"]


def _apply_arm(reach_axis, grip_axis):
    # J1+J2 ALTIJD samen (botsingsrandvoorwaarde) -- zie de kop van dit bestand.
    if abs(reach_axis) < 0.05:
        _freeze(1)
        _freeze(2)
    else:
        doel = 180.0 if reach_axis > 0 else 0.0   # omhoog = strekken (weg van rust)
        snelheid = abs(reach_axis) * ARM_MAX_SPEED_DEG_S
        sc.servo_pos(1, doel, snelheid)
        sc.servo_pos(2, doel, snelheid)

    if abs(grip_axis) < 0.05:
        _freeze(4)
    else:
        doel = 0.0 if grip_axis > 0 else 180.0    # rechts = open (0 graden = open, zie geometry.py)
        snelheid = abs(grip_axis) * ARM_MAX_SPEED_DEG_S
        sc.servo_pos(4, doel, snelheid)


# ----------------------------------------------------------------
# Achtergrondtaak: rijden + arm + deadman + obstakelstop (ALTIJD actief,
# onafhankelijk van een websocket-verbinding -- zie de kop van dit bestand)
# ----------------------------------------------------------------
async def control_task():
    while True:
        now = time.ticks_ms()
        stil = time.ticks_diff(now, _last_cmd_ms) > DEADMAN_MS
        _state["deadman_ms"] = time.ticks_diff(now, _last_cmd_ms)

        us_cm, us_kind = ultrasoon.read_cm()
        us_ok = (us_kind == 'ok')
        tof_cm = None
        tof_ok = False
        if _tof_ok and tof.data_ready():
            mm = tof.read(blocking=False)
            tof_ok = (tof.range_status == RANGE_VALID)
            if tof_ok:
                tof_cm = mm / 10.0
        elif _tof_ok and _state["tof_ok"]:
            # geen nieuwe meting deze tick: laatst bekende geldige waarde aanhouden
            tof_cm = _state["tof_cm"]
            tof_ok = True

        if stil or _scanning:
            fwd, turn, reach, grip = 0.0, 0.0, 0.0, 0.0
        else:
            fwd, turn, reach, grip = _cmd["fwd"], _cmd["turn"], _cmd["reach"], _cmd["grip"]
            # obstakelstop: alleen VERDER vooruit rijden blokkeren, nooit achteruit/draaien
            kortste = None
            if us_ok and us_cm is not None:
                kortste = us_cm
            if tof_ok and tof_cm is not None:
                kortste = tof_cm if kortste is None else min(kortste, tof_cm)
            if fwd > 0.0 and kortste is not None and kortste <= geo.STOP_DIST_CM:
                fwd = 0.0

        ld.set_target(fwd, turn)
        ld.service()
        _apply_arm(reach, grip)

        _state["us_cm"], _state["us_ok"] = us_cm, us_ok
        _state["tof_cm"], _state["tof_ok"] = tof_cm, tof_ok

        await asyncio.sleep_ms(sr.SLICE_MS // 2)


def _heading_deg():
    import math
    mx, my, _mz = sensor.magnetic
    return math.degrees(math.atan2(my, mx)) % 360.0


def _refresh_slow_telemetry():
    """Dingen die niet elke control-tick hoeven: kompas, LDR, servo-percentages,
    rijsnelheid. Wordt vanuit sender() aangeroepen, op TELEMETRY_HZ."""
    _state["heading"] = round(_heading_deg(), 1)
    pa, pb = ldr.read_synced()
    _state["ldr_a"], _state["ldr_b"] = pa, pb
    for nr, key in ((1, "j1_pct"), (2, "j2_pct"), (4, "grip_pct")):
        s = sc.servos[nr]
        rest_deg = sc._pct_to_deg(nr, sc.rest_pct[nr])
        _state[key] = round(100.0 * (s["pos_deg"] - rest_deg) / 180.0, 1)
    speed, turn = ld.status()
    _state["speed_cm_s"] = round(speed, 1)
    _state["turn_cm_s"] = round(turn, 1)


# ----------------------------------------------------------------
# Commando-afhandeling (vanuit de websocket-receiver)
# ----------------------------------------------------------------
def _handle_message(msg):
    global _last_cmd_ms
    try:
        data = json.loads(msg)
    except Exception:
        print("ongeldig bericht:", msg)
        return

    cmd = data.get("cmd")
    if cmd == "scan":
        _do_scan()
        return
    if cmd == "laser_toggle":
        _toggle_laser()
        return
    if cmd == "stop":
        ld.stop()
        _cmd["fwd"] = _cmd["turn"] = _cmd["reach"] = _cmd["grip"] = 0.0
        _last_cmd_ms = time.ticks_ms()
        return

    drive = data.get("drive")
    if drive:
        _cmd["fwd"] = max(-1.0, min(1.0, float(drive.get("fwd", 0.0))))
        _cmd["turn"] = max(-1.0, min(1.0, float(drive.get("turn", 0.0))))
    arm = data.get("arm")
    if arm:
        _cmd["reach"] = max(-1.0, min(1.0, float(arm.get("reach", 0.0))))
        _cmd["grip"] = max(-1.0, min(1.0, float(arm.get("grip", 0.0))))
    _last_cmd_ms = time.ticks_ms()


# ----------------------------------------------------------------
# Web-app
# ----------------------------------------------------------------
app = Microdot()

PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>DIY CAR</title>
<style>
 :root{--joy: min(30vh, 34vw)}
 html,body{height:100%;margin:0;background:#0e0e10;color:#ebebeb;font-family:sans-serif;
   overflow:hidden;user-select:none;-webkit-user-select:none;touch-action:none}
 body{display:flex;flex-direction:column;box-sizing:border-box;padding:1vh 1vw;gap:1vh}
 .panel{background:#1a1a1e;border:2px solid #323238;border-radius:12px;padding:0.8vh 1.2vw;
   box-sizing:border-box;min-width:0}
 .title{color:#999;font-size:clamp(9px,1.6vh,13px);letter-spacing:.05em;white-space:nowrap}
 #toprow{display:flex;justify-content:space-between;align-items:flex-start;gap:1vw;flex:0 0 auto}
 #topleft{flex:1 1 0}
 #topmid{flex:1.4 1 0;text-align:center}
 #topright{flex:1 1 0;text-align:center}
 .row{display:flex;justify-content:space-between;font-size:clamp(10px,1.8vh,15px);margin:2px 0}
 .big{font-weight:bold;font-size:clamp(13px,2.4vh,20px);color:#eee}
 .ldrrow{display:flex;justify-content:space-between;align-items:center;margin:2px 0}
 .pct{font-size:clamp(14px,3vh,26px);font-weight:bold;color:#0a6}
 #arrow{flex:1;height:clamp(16px,3vh,30px);margin:0 4px}
 .btnrow{display:flex;justify-content:space-between;margin-top:4px;gap:6px}
 button{background:#2a2a30;color:#eee;border:1px solid #46464e;border-radius:10px;
   font-size:clamp(11px,1.8vh,15px);font-weight:bold;padding:0.8vh 1vw;flex:1}
 button:active{background:#0a6}
 #laserbtn.aan{background:#664a10;border-color:#c89b3a;color:#f0c850}
 #compass{display:block;margin:0 auto;width:min(14vh,16vw);height:min(14vh,16vw)}
 #bottomrow{display:flex;justify-content:space-between;align-items:center;flex:1 1 auto;
   min-height:0;gap:1vw}
 .joywrap{width:var(--joy);text-align:center;flex:0 0 auto}
 .joy{width:var(--joy);height:var(--joy);border-radius:50%;background:#16161a;
   border:2px solid #323238;position:relative;touch-action:none}
 .knob{width:28%;height:28%;border-radius:50%;background:#0a6;position:absolute;
   left:50%;top:50%;transform:translate(-50%,-50%);box-shadow:0 0 0 2px #063}
 .joylabel{position:absolute;color:#777;font-size:clamp(9px,1.5vh,12px)}
 .readout{margin-top:4px;font-size:clamp(10px,1.6vh,14px);color:#ddd}
 #stopbtn{flex:0 0 auto;width:min(22vw,140px);height:min(10vh,56px);background:#780f0f;
   border:3px solid #d22d2d;color:#ffe6e6;font-size:clamp(14px,2.6vh,22px);border-radius:12px}
 #stopbtn:active{background:#d22d2d}
 #status{position:fixed;right:6px;bottom:2px;font-size:10px;color:#777}
 #status.verbonden{color:#0a6}
</style></head><body>

<div id="toprow">
<div class="panel" id="topleft">
  <div class="title">AFSTAND</div>
  <div class="row"><span>ultrasoon</span><span class="big" id="us">-- cm</span></div>
  <div class="row"><span>ToF</span><span class="big" id="tof">-- cm</span></div>
</div>

<div class="panel" id="topmid">
  <div class="title">LDR / STUURRICHTING</div>
  <div class="ldrrow">
    <div><div>L</div><div class="pct" id="ldra">--%</div></div>
    <svg id="arrow" viewBox="0 0 200 30"><line id="arrowline" x1="40" y1="15" x2="160" y2="15"
         stroke="#e6aa1e" stroke-width="5"/><polygon id="arrowhead" points="160,5 160,25 180,15"
         fill="#e6aa1e"/></svg>
    <div><div>R</div><div class="pct" id="ldrb">--%</div></div>
  </div>
  <div class="btnrow">
    <button id="scanbtn">SCAN</button>
    <button id="laserbtn">LASER AAN</button>
  </div>
</div>

<div class="panel" id="topright">
  <div class="title">KOMPAS</div>
  <canvas id="compass" width="160" height="160"></canvas>
  <div class="big" id="heading">--&deg;</div>
</div>
</div>

<div id="bottomrow">
<div class="joywrap" id="armwrap">
  <div class="title">ARM / GRIJPER</div>
  <div class="joy" id="armjoy">
    <div class="joylabel" style="top:4px;left:50%;transform:translateX(-50%)">strekken</div>
    <div class="joylabel" style="bottom:4px;left:50%;transform:translateX(-50%)">intrekken</div>
    <div class="joylabel" style="left:6px;top:50%;transform:translateY(-50%)">dicht</div>
    <div class="joylabel" style="right:6px;top:50%;transform:translateY(-50%)">open</div>
    <div class="knob" id="armknob"></div>
  </div>
  <div class="readout" id="armreadout">J1 0% &nbsp; J2 0% &nbsp; grijper 0%</div>
</div>

<button id="stopbtn">STOP</button>

<div class="joywrap" id="drivewrap">
  <div class="title">RIJDEN</div>
  <div class="joy" id="drivejoy">
    <div class="joylabel" style="top:4px;left:50%;transform:translateX(-50%)">vooruit</div>
    <div class="joylabel" style="bottom:4px;left:50%;transform:translateX(-50%)">achteruit</div>
    <div class="joylabel" style="left:6px;top:50%;transform:translateY(-50%)">bocht links</div>
    <div class="joylabel" style="right:6px;top:50%;transform:translateY(-50%)">bocht rechts</div>
    <div class="knob" id="driveknob"></div>
  </div>
  <div class="readout" id="drivereadout">0.0 cm/s &nbsp; 0.0&deg;/s</div>
</div>
</div>

<div id="status">verbinden...</div>

<script>
let ws;
let drive = {fwd:0, turn:0};
let arm = {reach:0, grip:0};

function connect(){
  ws = new WebSocket("ws://" + location.host + "/ws");
  const st = document.getElementById('status');
  ws.onopen = () => { st.textContent = "verbonden"; st.className = "verbonden"; };
  ws.onclose = () => { st.textContent = "verbroken"; st.className = ""; setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => render(JSON.parse(e.data));
}

function sendDrive(){
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({drive, arm}));
}
setInterval(sendDrive, 100);   // continue stroom i.p.v. alleen bij verandering (deadman!)

function sendCmd(cmd){
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({cmd}));
}

// ---------------- Joystick-logica (herbruikbaar voor beide) ----------------
function setupJoystick(joyId, knobId, onChange){
  const joy = document.getElementById(joyId);
  const knob = document.getElementById(knobId);
  let active = false;

  function setKnob(dx, dy){
    knob.style.transform = "translate(calc(-50% + " + dx + "px), calc(-50% + " + dy + "px))";
  }

  function fromEvent(ev){
    const rect = joy.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const r = rect.width / 2 - 23;   // max. uitslag = joystick-straal minus halve knopbreedte (28%)
    const t = ev.touches ? ev.touches[0] : ev;
    let dx = t.clientX - cx, dy = t.clientY - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > r){ dx = dx / dist * r; dy = dy / dist * r; }
    setKnob(dx, dy);
    onChange(dx / r, -dy / r);   // y omgekeerd: omhoog = positief
  }

  function release(){
    active = false;
    setKnob(0, 0);
    onChange(0, 0);
  }

  joy.addEventListener('pointerdown', (ev) => { active = true; joy.setPointerCapture(ev.pointerId); fromEvent(ev); });
  joy.addEventListener('pointermove', (ev) => { if (active) fromEvent(ev); });
  joy.addEventListener('pointerup', release);
  joy.addEventListener('pointercancel', release);
}

setupJoystick('drivejoy', 'driveknob', (x, y) => { drive.fwd = y; drive.turn = x; sendDrive(); });
setupJoystick('armjoy', 'armknob', (x, y) => { arm.reach = y; arm.grip = x; sendDrive(); });

document.getElementById('scanbtn').onclick = () => sendCmd('scan');
document.getElementById('laserbtn').onclick = () => sendCmd('laser_toggle');
document.getElementById('stopbtn').onclick = () => sendCmd('stop');

// ---------------- Kompas ----------------
const cv = document.getElementById('compass');
const ctx = cv.getContext('2d');
const DIRS = [["N",0],["NO",45],["O",90],["ZO",135],["Z",180],["ZW",225],["W",270],["NW",315]];

function drawCompass(heading){
  const cx = 80, cy = 80, R = 70;
  ctx.clearRect(0, 0, 160, 160);
  ctx.strokeStyle = "#5a5a62"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.stroke();
  ctx.font = "13px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (const [label, ang] of DIRS){
    const rad = (ang - 90) * Math.PI / 180;
    const tx = cx + (R - 16) * Math.cos(rad), ty = cy + (R - 16) * Math.sin(rad);
    ctx.fillStyle = (label === "N") ? "#d22d2d" : "#999";
    ctx.fillText(label, tx, ty);
  }
  const rad = (heading - 90) * Math.PI / 180;
  const nx = cx + (R - 20) * Math.cos(rad), ny = cy + (R - 20) * Math.sin(rad);
  const lx = cx + 8 * Math.cos(rad + Math.PI / 2), ly = cy + 8 * Math.sin(rad + Math.PI / 2);
  const rx = cx + 8 * Math.cos(rad - Math.PI / 2), ry = cy + 8 * Math.sin(rad - Math.PI / 2);
  ctx.fillStyle = "#e6aa1e";
  ctx.beginPath(); ctx.moveTo(nx, ny); ctx.lineTo(lx, ly); ctx.lineTo(rx, ry); ctx.closePath(); ctx.fill();
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, 7); ctx.fill();
}

// ---------------- Telemetrie renderen ----------------
function render(d){
  document.getElementById('us').textContent = d.us_ok ? d.us_cm.toFixed(1) + " cm" : "--";
  document.getElementById('tof').textContent = d.tof_ok ? d.tof_cm.toFixed(1) + " cm" : "--";
  document.getElementById('ldra').textContent = d.ldr_a.toFixed(0) + "%";
  document.getElementById('ldrb').textContent = d.ldr_b.toFixed(0) + "%";

  const diff = d.ldr_b - d.ldr_a;   // >0: R feller -> stuur naar rechts
  const head = document.getElementById('arrowhead');
  const line = document.getElementById('arrowline');
  const mid = 100 + Math.max(-60, Math.min(60, diff));
  line.setAttribute('x2', mid); head.setAttribute('points', mid + ',5 ' + mid + ',25 ' + (mid + 20) + ',15');

  document.getElementById('laserbtn').className = d.laser ? 'aan' : '';
  document.getElementById('laserbtn').textContent = d.laser ? 'LASER UIT' : 'LASER AAN';

  document.getElementById('heading').textContent = d.heading.toFixed(0) + "°";
  drawCompass(d.heading);

  document.getElementById('armreadout').textContent =
    "J1 " + d.j1_pct.toFixed(0) + "%   J2 " + d.j2_pct.toFixed(0) + "%   grijper " + d.grip_pct.toFixed(0) + "%";
  document.getElementById('drivereadout').textContent =
    d.speed_cm_s.toFixed(1) + " cm/s   " + d.turn_cm_s.toFixed(1) + " cm/s bocht";
}

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
            _refresh_slow_telemetry()
            await ws.send(json.dumps(_state))
            await asyncio.sleep_ms(1000 // TELEMETRY_HZ)

    async def receiver():
        while True:
            msg = await ws.receive()
            _handle_message(msg)

    try:
        await asyncio.gather(sender(), receiver())
    except Exception as e:
        print("websocket gesloten:", e)


# ----------------------------------------------------------------
# Start
# ----------------------------------------------------------------
async def main():
    asyncio.create_task(control_task())
    print("webserver start op poort 80 ...")
    await app.start_server(port=80)


start_ap()
asyncio.run(main())
