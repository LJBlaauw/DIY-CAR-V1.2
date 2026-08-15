"""Hardwaretest voor de data-logger uit data_logging_bijgewerkt.md.

Alle sensor- en regelwaarden worden gesimuleerd. Het script is bedoeld voor
MicroPython op een Pico W/Pico 2 W met ``microdot.py`` in ``/lib``.

Gebruik:
    1. Start dit bestand op de Pico.
    2. Verbind de pc met wifi ``DIYCAR`` (wachtwoord ``diycar12345``).
    3. curl http://192.168.4.1/start?run_id=testrit-1
    4. curl --fail --no-buffer http://192.168.4.1/log.csv -o testrit.csv
    5. curl http://192.168.4.1/stop

``/status`` toont de actuele tellers. Een tweede gelijktijdige logclient krijgt
HTTP 409. Stop curl met Ctrl-C om reconnectgedrag te testen; de run blijft dan
actief.
"""

import asyncio
import math
import time

import machine
import network
import ubinascii
from microdot import Microdot, Response


AP_SSID = "DIYCAR"
AP_PASSWORD = "diycar12345"
LOG_PERIOD_MS = 100             # 10 Hz; gebruik 20 voor een korte 50 Hz-test

CSV_HEADER = (
    b"run_id;boot_id;seq;t_ms;state;ldr_a_u16;ldr_b_u16;ldr_sid;"
    b"us_cm;us_sid;travel_a_steps;travel_b_steps;odom_heading_deg;"
    b"turn_setpoint_dps;gyro_z_dps;gyro_sid;gyro_t_ms;turn_applied_dps;"
    b"ring_dropped;log_missed;disconnects\n"
)


class RecordRing:
    """Niet-blokkerende byte-ring die uitsluitend complete records uitleest."""

    def __init__(self, size=8192, max_records=96):
        self.buf = bytearray(size)
        self.mv = memoryview(self.buf)
        self.size = size
        self.lengths = [0] * max_records
        self.max_records = max_records
        self.r = self.w = self.used = 0
        self.lr = self.lw = self.records = 0
        self.dropped = 0

    def clear(self, reset_dropped=False):
        self.r = self.w = self.used = 0
        self.lr = self.lw = self.records = 0
        if reset_dropped:
            self.dropped = 0

    def put(self, data):
        n = len(data)
        if n == 0:
            return True
        if (n > self.size or n > self.size - self.used or
                self.records >= self.max_records):
            self.dropped += 1
            return False

        end = self.w + n
        if end <= self.size:
            self.mv[self.w:end] = data
        else:
            first = self.size - self.w
            self.mv[self.w:] = data[:first]
            self.mv[:n - first] = data[first:]

        self.lengths[self.lw] = n
        self.lw = (self.lw + 1) % self.max_records
        self.records += 1
        self.used += n
        self.w = end % self.size
        return True

    def take_record(self):
        if self.records == 0:
            return None

        n = self.lengths[self.lr]
        end = self.r + n
        if end <= self.size:
            data = bytes(self.mv[self.r:end])
        else:
            first = self.size - self.r
            out = bytearray(n)
            out[:first] = self.mv[self.r:]
            out[first:] = self.mv[:n - first]
            data = bytes(out)

        self.lr = (self.lr + 1) % self.max_records
        self.records -= 1
        self.used -= n
        self.r = end % self.size
        return data


LOG = RecordRing()
LOG_STATS = {"log_missed": 0, "disconnects": 0}
RUN = {
    "active": False,
    "run_id": "",
    "boot_id": ubinascii.hexlify(machine.unique_id()).decode(),
    "t0": 0,
    "seq": 0,
}
LOG_CLIENT_ACTIVE = False

LOG_STATE = {
    "state": "idle",
    "ldr_a_u16": 0,
    "ldr_b_u16": 0,
    "ldr_sid": 0,
    "us_cm": None,
    "us_sid": 0,
    "travel_a_steps": 0,
    "travel_b_steps": 0,
    "odom_heading_deg": 0.0,
    "turn_setpoint_dps": 0.0,
    "gyro_z_dps": 0.0,
    "gyro_sid": 0,
    "gyro_t_ms": 0,
    "turn_applied_dps": 0.0,
}


def log_snapshot():
    s = LOG_STATE
    return (
        s["state"], s["ldr_a_u16"], s["ldr_b_u16"], s["ldr_sid"],
        s["us_cm"], s["us_sid"], s["travel_a_steps"],
        s["travel_b_steps"], s["odom_heading_deg"],
        s["turn_setpoint_dps"], s["gyro_z_dps"], s["gyro_sid"],
        s["gyro_t_ms"], s["turn_applied_dps"],
    )


def valid_id(value):
    if not value or len(value) > 40:
        return False
    for char in value:
        if not (char.isalnum() or char in "_-"):
            return False
    return True


def start_run(run_id):
    global LOG_CLIENT_ACTIVE
    LOG.clear(reset_dropped=True)
    LOG_STATS["log_missed"] = 0
    LOG_STATS["disconnects"] = 0
    LOG_CLIENT_ACTIVE = False
    RUN["run_id"] = run_id
    RUN["t0"] = time.ticks_ms()
    RUN["seq"] = 0
    RUN["active"] = True


def stop_run():
    RUN["active"] = False


async def simulated_sensor_task():
    """Publiceert op circa 50 Hz coherente, herkenbaar veranderende waarden."""
    sid = 0
    while True:
        sid += 1
        phase = sid * 0.08
        turn = 25.0 * math.sin(phase * 0.35)
        left_step = 2 + (1 if turn > 8 else 0)
        right_step = 2 + (1 if turn < -8 else 0)

        s = LOG_STATE
        s["state"] = "driving" if RUN["active"] else "idle"
        s["ldr_a_u16"] = int(32000 + 18000 * math.sin(phase))
        s["ldr_b_u16"] = int(32000 + 18000 * math.sin(phase + 0.7))
        s["ldr_sid"] = sid

        # Regelmatig None om het verplichte lege CSV-veld te testen.
        s["us_cm"] = None if sid % 75 < 5 else 35.0 + 8.0 * math.sin(phase * 0.2)
        s["us_sid"] = sid
        s["travel_a_steps"] += left_step
        s["travel_b_steps"] += right_step
        s["odom_heading_deg"] = ((s["odom_heading_deg"] + turn * 0.02 + 180) % 360) - 180
        s["turn_setpoint_dps"] = turn
        s["gyro_z_dps"] = turn + 0.8 * math.sin(phase * 2.3)
        s["gyro_sid"] = sid
        s["gyro_t_ms"] = time.ticks_ms()
        s["turn_applied_dps"] = turn * 0.92
        await asyncio.sleep_ms(20)


async def log_task():
    next_ms = time.ticks_ms()
    while True:
        if not RUN["active"]:
            await asyncio.sleep_ms(20)
            next_ms = RUN["t0"]
            continue

        now = time.ticks_ms()
        delay = time.ticks_diff(next_ms, now)
        if delay > 0:
            await asyncio.sleep_ms(delay)
            continue

        late_ms = time.ticks_diff(now, next_ms)
        if late_ms >= LOG_PERIOD_MS:
            missed = late_ms // LOG_PERIOD_MS
            RUN["seq"] += missed
            LOG_STATS["log_missed"] += missed
            next_ms = time.ticks_add(next_ms, missed * LOG_PERIOD_MS)

        RUN["seq"] += 1
        seq = RUN["seq"]
        (state, ldr_a, ldr_b, ldr_sid, us_cm, us_sid, travel_a,
         travel_b, heading, turn_setpoint, gyro_z, gyro_sid, gyro_t_ms,
         turn_applied) = log_snapshot()
        us_text = "" if us_cm is None else "%.1f" % us_cm

        row = (
            "%s;%s;%d;%d;%s;%d;%d;%d;%s;%d;%d;%d;%.2f;%.2f;%.2f;%d;%d;%.2f;%d;%d;%d\n"
            % (RUN["run_id"], RUN["boot_id"], seq,
               time.ticks_diff(time.ticks_ms(), RUN["t0"]), state,
               ldr_a, ldr_b, ldr_sid, us_text, us_sid, travel_a, travel_b,
               heading, turn_setpoint, gyro_z, gyro_sid, gyro_t_ms,
               turn_applied, LOG.dropped, LOG_STATS["log_missed"],
               LOG_STATS["disconnects"])
        ).encode()
        LOG.put(row)
        next_ms = time.ticks_add(next_ms, LOG_PERIOD_MS)
        await asyncio.sleep_ms(0)


class LogStream:
    def __init__(self, ring):
        self.ring = ring
        self.header_pending = True
        self.closed = False

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.header_pending:
            self.header_pending = False
            return CSV_HEADER
        while True:
            row = self.ring.take_record()
            if row is not None:
                return row
            if not RUN["active"]:
                raise StopAsyncIteration
            await asyncio.sleep_ms(20)

    async def aclose(self):
        global LOG_CLIENT_ACTIVE
        if self.closed:
            return
        self.closed = True
        LOG_CLIENT_ACTIVE = False
        if RUN["active"]:
            LOG_STATS["disconnects"] += 1


app = Microdot()


@app.route("/")
async def index(request):
    return (
        "DIY CAR data-loggingtest\n"
        "GET /start?run_id=testrit-1\nGET /log.csv\nGET /status\nGET /stop\n",
        200,
        {"Content-Type": "text/plain; charset=utf-8"},
    )


@app.route("/start")
async def start_endpoint(request):
    run_id = request.args.get("run_id", "test-run")
    if not valid_id(run_id):
        return "ongeldige run_id; gebruik letters, cijfers, _ of -\n", 400
    if RUN["active"]:
        return "er is al een actieve run\n", 409
    if LOG_CLIENT_ACTIVE:
        return "wacht tot de vorige logstream volledig is afgesloten\n", 409
    start_run(run_id)
    return "run gestart: %s\n" % run_id


@app.route("/stop")
async def stop_endpoint(request):
    if not RUN["active"]:
        return "geen actieve run\n", 409
    stop_run()
    return "run gestopt; resterende records kunnen nog worden uitgelezen\n"


@app.route("/status")
async def status_endpoint(request):
    return {
        "active": RUN["active"],
        "run_id": RUN["run_id"],
        "boot_id": RUN["boot_id"],
        "seq": RUN["seq"],
        "ring_records": LOG.records,
        "ring_bytes": LOG.used,
        "ring_dropped": LOG.dropped,
        "log_missed": LOG_STATS["log_missed"],
        "disconnects": LOG_STATS["disconnects"],
        "client_active": LOG_CLIENT_ACTIVE,
    }


@app.route("/log.csv")
async def log_csv(request):
    global LOG_CLIENT_ACTIVE
    if LOG_CLIENT_ACTIVE:
        return Response("log client already connected\n", status_code=409)
    if not RUN["active"] and LOG.records == 0:
        return Response("no active run\n", status_code=409)
    LOG_CLIENT_ACTIVE = True
    return Response(
        body=LogStream(LOG),
        headers={
            "Content-Type": "text/csv; charset=utf-8",
            "Content-Disposition": 'attachment; filename="run.csv"',
            "Cache-Control": "no-store",
        },
    )


def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ap.active(True)
    while not ap.active():
        time.sleep_ms(20)
    print("AP actief:", AP_SSID, "->", ap.ifconfig()[0])
    return ap


async def main():
    start_ap()
    asyncio.create_task(simulated_sensor_task())
    asyncio.create_task(log_task())
    print("Open http://192.168.4.1/ en start eerst een run")
    await app.start_server(host="0.0.0.0", port=80)


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
