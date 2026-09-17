# Data logging

## Target

During a measurement run, sensor values, control data and vehicle status are stored on a
PC without stopping real-time engine control.

The most important design rule is:

> Losing log lines is acceptable; missing deadlines in the
> motor or control loop does not.

Therefore, the control loop never writes directly to WiFi or flash. The logger
places complete CSV lines in a limited RAM buffer. A separate
HTTP stream sends the content when the network connection has room for
.

## Chosen approach

The preferred solution consists of:

1. a shared snapshot with the last known sensor and control values;
2. a logging task that converts this snapshot to CSV at 10 Hz by default;
3. a bounded, record-aware ring buffer in RAM;
4. an explicit run status (`start`, `active`, `stop`);
5. a separate Microdot route `/log.csv`;
6. `curl` on the PC, which writes the stream directly to a file;
7. a small event log on flash as a safety net for already recorded
state changes and errors.

The existing web socket remains for control and slow
UI telemetry. Measurement data and UI data are not merged into the same stream
.

A **run** and an **HTTP connection** are deliberately two different things:

- starting a run clears the RAM buffer, resets the run counters and
chooses a new `run_id`;
- `/log.csv` only reads data from the active run;
- a broken HTTP connection does not stop the run;
- reconnecting to `/log.csv` returns a header and continues at
the next full log line;
- stopping a run stops production of new CSV lines. Any
remaining lines may then be read out.

As a result, the PC or web interface explicitly determines when a measurement run starts and
ends, without a short WiFi failure automatically starting a new run.

```text
sensor- en regeltaken
        |
        v
 gedeelde snapshot
        |
   logger op 10 Hz
        |
        v
 record-aware RAM-ring ----> /log.csv ----> curl ----> CSV-bestand op pc
        |                         |
        +--> ring_dropped         +--> disconnects
        +--> log_missed

toestandswisselingen en fouten ----> klein eventlog op flash
```

## Why HTTP Streaming

HTTP streaming is the best fit for this project because Microdot is already used for the
web interface and the PC does not require any special client code.
Furthermore, the PC determines when recording starts and ends, and `curl` writes the
data directly to disk.

Benefits:

- no websocket framing or separate Python client;
- a separate connection for measurement data;
- CSV can be used immediately during and after the ride;
- reconnecting is easy;
- a lost connection should never block the control loop.

## Frequencies

Use different frequencies for control, logging and presentation:

| Function | Guide value | Note |
|---|---:|---|
| `Move.service()` and quick control | >= 100Hz | At least once every 10 ms; see `globale_specificatie.md` |
| Gyro and inner control loop | approximately 50 Hz | Do not decrease to simplify logging |
| CSV logging normal | 10Hz | Standard for complete measuring runs |
| CSV logging control tuning | 50Hz | Only for short tuning tests; Validate CPU/heap first |
| UI Telemetry | 2-5Hz | Human presentation values ​​only |
| Events | directly | Only state changes, results and errors |

The 100 Hz for `Move.service()` is not a guideline value but a strict requirement: the
FIFO runway is 60 ms and the bridge segment at the end of the ramp exists correctly
because 4 FIFO words at top speed together only amount to approximately 3 ms. Logging is allowed
So never reduce the call frequency.

The log task only reads existing snapshots. It does not start
ultrasonic measurement, ADC conversion sequence, compass fusion or control calculation itself.

10 Hz is sufficient for trajectory, sensor and condition analysis. For tuning
the approximately 50 Hz gyro inner loop, 10 Hz is too low to properly see fast oscillations and
short course disturbances. To do this, temporarily use the
50 Hz profile and check on the real Pico that WiFi, heap and garbage
collection do not affect the motor control.

## CSV format

Use semicolons as separators and decimal points for floats. Each
HTTP stream starts with one header.

Recommended schedule:

```text
run_id;boot_id;seq;t_ms;state;ldr_a_u16;ldr_b_u16;ldr_sid;us_cm;us_sid;travel_a_steps;travel_b_steps;odom_heading_deg;turn_setpoint_dps;gyro_z_dps;gyro_sid;gyro_t_ms;turn_applied_dps;ring_dropped;log_missed;disconnects
```

Meaning of the most important fields:

- `run_id`: identification of the current measurement trip;
- `boot_id`: changes with every restart of the Pico;
- `seq`: number of the **nominal sample slot**; also walks through missed
log deadlines and ring overflow;
- `t_ms`: time since the start of the measurement run;
- `ldr_a_u16`, `ldr_b_u16`: MicroPython `ADC.read_u16()` values ​​in the range
0...65535; so these are not directly the 12-bit hardware counts;
- `ldr_sid`: sequence ID of the last available LDR measurement;
- `us_cm`: last known ultrasonic distance in cm with one decimal place. The field is
**empty** when no valid measurement is available (`None` in the snapshot).
An empty field is therefore not 0 cm and must be treated as a missing value
when reading;
- `us_sid`: sequence ID of the last available ultrasound measurement;
- `travel_a_steps`, `travel_b_steps`: **signed** wheel positions; not the monotonous
PIO pulse counters that do not know the DIR pin;
- `odom_heading_deg`: course from the signed cycling odometry; the name makes it explicit
that this is not a magnetometer heading or integrated gyro angle;
- `gyro_sid`, `gyro_t_ms`: identity and measurement time of the gyro value, so that
remains visible how old that value was when the 10 Hz logger took the snapshot;
- `ring_dropped`: cumulative number of complete log records that could not be added due to a full
RAM ring;
- `log_missed`: cumulative number of nominal logging slots deliberately skipped due to a late
logger deadline;
- `disconnects`: Number of logstream connections dropped during the run.

A jump in `seq` shows that nominal samples are missing. `ring_dropped`
declares loss before network stream; `log_missed` explains skipped
logger deadlines. A network disconnect can also cost a record that has already been removed from the
ring. That loss is not reliable as `ring_dropped` to
measured; `disconnects` and a `seq` jump in the following file do make
visible.

After reconnecting, the new CSV file starts over with the header, but
continues to cycle through `run_id`, `boot_id`, and `seq` as long as the same run is active.

### Text fields

To avoid a generic CSV quote/escape routine in the real-time logger,
, text fields are kept limited:

- `run_id` and `boot_id`: only letters, numbers, `_` and `-`;
- `state`: fixed enum from the application, also without `;`, CR or LF.

This allows creating any log line with simple `%` formatting
without ambiguous CSV format.

## RAM ring buffer

The buffer is bounded and `put()` never waits. If a complete CSV line does not fit
, that line is discarded entirely and increases to `ring_dropped`.

The ring is **record-aware**: the record lengths are kept separately. The
consumer always deletes exactly one full CSV line. That is important
with reconnects. A byte ring that randomly removes 1024 bytes can be put in the middle of
in a CSV line; after a disconnect, a new file could start with a
header plus the second half of that old line.

```python
# lib/log/ring.py
class RecordRing:
    """Bounded byte ring for complete log records.

    put() does not block. If bytes or a record slot are unavailable, the
    complete record is discarded and dropped is incremented.

    take_record() always returns exactly one complete record. Conversion to
    bytes allocates an object; at 10 Hz this is deliberately accepted and must
    be validated on the hardware.

    The wrap-around branch of put() also allocates two temporary slices. This
    can be avoided with memoryview(data) if heap measurements warrant it.
    """

    def __init__(self, size=8192, max_records=96):
        self.buf = bytearray(size)
        self.mv = memoryview(self.buf)
        self.size = size

        # Preallocated ring of record lengths. A Python list uses more RAM than
        # uint16, but does not grow during normal operation. Optimize only if
        # heap measurements show that this is necessary.
        self.lengths = [0] * max_records
        self.max_records = max_records

        self.r = 0
        self.w = 0
        self.used = 0

        self.lr = 0
        self.lw = 0
        self.records = 0
        self.dropped = 0

    def clear(self, reset_dropped=False):
        self.r = 0
        self.w = 0
        self.used = 0
        self.lr = 0
        self.lw = 0
        self.records = 0
        if reset_dropped:
            self.dropped = 0

    def put(self, data):
        n = len(data)
        if n == 0:
            return True

        if n > self.size:
            self.dropped += 1
            return False

        if n > self.size - self.used or self.records >= self.max_records:
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
```

A log line is approximately 115-140 bytes with realistic values. At 10 Hz this is
approximately 1.2-1.5 kB/s and therefore bridges an 8 kB byte buffer for more than five seconds.
`max_records=96` gives 9.6 seconds at 10 Hz, so the byte limit binds first.
Check actual average line length and heap on hardware.

With the 50 Hz tuning profile, this runway **does not apply**: 8 kB is then only
approximately 1.2-1.4 seconds and `max_records=96` only 1.9 seconds. Every
network interruption of more than approximately one second immediately results in
`ring_dropped` at 50 Hz. For that profile, consciously increase `size` and `max_records`, or
explicitly accept the loss.

The ring is only short-term insurance against network delays, not
storage for a full ride. If a fault lasts longer, new
records are rejected and `ring_dropped` increases.

### Network loss after `take_record()`

Once Microdot receives a record from `take_record()`, that record is removed from
the ring. If the connection is lost exactly after that, it cannot be proven
that the PC received all bytes of that record. Thanks to the record-aware ring, a reconnect
always starts on a **new complete
CSV line**, but one or more complete lines may be missing. Therefore,
`seq` and `disconnects` remain necessary.

## Shared snapshot

Sensor and control tasks update their own state. The logger only reads the
last published values ​​and does not start new measurements or
control calculations itself.

Example as long as all writers run on the same MicroPython core:

```python
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
    # No await and no sensor-function calls in this function.
    s = LOG_STATE
    return (
        s["state"],
        s["ldr_a_u16"], s["ldr_b_u16"], s["ldr_sid"],
        s["us_cm"], s["us_sid"],
        s["travel_a_steps"], s["travel_b_steps"],
        s["odom_heading_deg"], s["turn_setpoint_dps"],
        s["gyro_z_dps"], s["gyro_sid"], s["gyro_t_ms"],
        s["turn_applied_dps"],
    )
```

Do not call `ultrasoon.read_cm()`, an ADC measurement loop or
`HeadingController.output()` again from the logger. Such functions can influence a measurement or
control calculation. The logger only observes.

### Multicore boundary condition

The above dictionary is **not a guaranteed coherent snapshot** when
core 1 updates multiple fields simultaneously. One CSV line can then combine, for example,
, a new gyro value, and an old `turn_applied_dps`.

Before core 1 can directly publish log fields, the core transfer
must be explicitly designed, for example with a double-buffer/mailbox or a
version/seqlock-like protocol. The real-time control loop should never have to wait for a
logger lock. Until that transfer has been validated, the following applies: publish the
snapshot from one core.

## Periodic logging task

An absolute deadline prevents normal period drift, but missed periods are allowed
**not to be overtaken with a burst**. If the logger has more than one period
is late, the intermediate nominal sample slots are deliberately skipped.
This is in accordance with the main rule: loss of measurement data before the control loop
gets tax.

The precise field names are examples and must be linked to the
final application upon integration.

```python
import asyncio
import time

LOG = RecordRing(8192, max_records=96)
LOG_PERIOD_MS = 100

LOG_STATS = {
    "log_missed": 0,
    "disconnects": 0,
}

RUN = {
    "active": False,
    "run_id": "",
    "boot_id": "",
    "t0": 0,
    "seq": 0,
}


def start_run(run_id, boot_id):
    # Call from the application/command task, not from a motor IRQ.
    global LOG_CLIENT_ACTIVE

    LOG.clear(reset_dropped=True)
    LOG_STATS["log_missed"] = 0
    LOG_STATS["disconnects"] = 0

    # Safeguard: if a previous stream never reached aclose(), a new run must not
    # start with a permanently locked log client.
    # See the Microdot section for LOG_CLIENT_ACTIVE.
    LOG_CLIENT_ACTIVE = False

    RUN["run_id"] = run_id
    RUN["boot_id"] = boot_id
    RUN["t0"] = time.ticks_ms()
    RUN["seq"] = 0
    RUN["active"] = True


def stop_run():
    RUN["active"] = False


async def log_task():
    next_ms = time.ticks_ms()

    while True:
        if not RUN["active"]:
            # For a new run, establish the phase again from its t0.
            await asyncio.sleep_ms(20)
            next_ms = RUN["t0"]
            continue

        now = time.ticks_ms()
        delay = time.ticks_diff(next_ms, now)
        if delay > 0:
            await asyncio.sleep_ms(delay)
            continue

        # If we are >= one full period late, skip those nominal slots.
        late_ms = time.ticks_diff(now, next_ms)
        if late_ms >= LOG_PERIOD_MS:
            missed = late_ms // LOG_PERIOD_MS
            RUN["seq"] += missed
            LOG_STATS["log_missed"] += missed
            next_ms = time.ticks_add(next_ms, missed * LOG_PERIOD_MS)

        RUN["seq"] += 1
        seq = RUN["seq"]

        (state, ldr_a, ldr_b, ldr_sid, us_cm, us_sid,
         travel_a, travel_b, odom_heading, turn_setpoint,
         gyro_z, gyro_sid, gyro_t_ms, turn_applied) = log_snapshot()

        us_text = "" if us_cm is None else "%.1f" % us_cm
        t_ms = time.ticks_diff(time.ticks_ms(), RUN["t0"])

        row = (
            "%s;%s;%d;%d;%s;%d;%d;%d;%s;%d;%d;%d;%.2f;%.2f;%.2f;%d;%d;%.2f;%d;%d;%d\n"
            % (
                RUN["run_id"],
                RUN["boot_id"],
                seq,
                t_ms,
                state,
                ldr_a,
                ldr_b,
                ldr_sid,
                us_text,
                us_sid,
                travel_a,
                travel_b,
                odom_heading,
                turn_setpoint,
                gyro_z,
                gyro_sid,
                gyro_t_ms,
                turn_applied,
                LOG.dropped,
                LOG_STATS["log_missed"],
                LOG_STATS["disconnects"],
            )
        ).encode()

        LOG.put(row)
        next_ms = time.ticks_add(next_ms, LOG_PERIOD_MS)

        # Altijd minstens één schedulerkans na formatteren/kopiëren.
        await asyncio.sleep_ms(0)
```

### Why `seq` is incremented before buffer

`seq` belongs to the nominal measurement slot, not to a successfully saved record.
If `LOG.put()` fails, that number is missing from the file. The same applies to
for deliberately skipped deadlines: `seq` jumps forward with the number of missed
slots. This makes it visible afterwards **when** data is missing.

CSV formatting and `encode()` allocate memory. This is usually at 10 Hz
well manageable, but validate this on the Pico while WiFi, gyro and motors
be active at the same time. Schedule garbage collection only on a measured safe basis
moment; don't run it randomly from the control loop.

## Microdot stream

### MicroPython limitation

Microdot accepts any object with a `__anext__` method as a response body and
uses that object directly as an async iterator (see `body_iter()` in
`lib/microdot/microdot.py`). A class-based async iterator is therefore the
safest form on MicroPython: the stream can wait when the ring is empty
without occupying the event loop.

Important for the cleanup: `Response.write()` calls `await iter.aclose()`
if the body has that method, both after a normal end of the iterator and
when a `awrite()` fails with a socket error from `MUTED_SOCKET_ERRORS`
(32, 54, 104, 128). So that's the hook for disconnect cleanup. At one
`OSError` outside that list will not call `aclose()`; there is one for that
additional safety net required (see below).

Revalidate this lifecycle when the Microdot version in `lib/microdot/`
is updated.

### Implementation

```python
import asyncio
from microdot import Response

CSV_HEADER = (
    b"run_id;boot_id;seq;t_ms;state;ldr_a_u16;ldr_b_u16;ldr_sid;"
    b"us_cm;us_sid;travel_a_steps;travel_b_steps;odom_heading_deg;"
    b"turn_setpoint_dps;gyro_z_dps;gyro_sid;gyro_t_ms;turn_applied_dps;"
    b"ring_dropped;log_missed;disconnects\n"
)


LOG_CLIENT_ACTIVE = False


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

            # Once the run has stopped and the ring is empty, the response may end.
            if not RUN["active"]:
                raise StopAsyncIteration

            await asyncio.sleep_ms(20)

    async def aclose(self):
        # Microdot calls this at normal iterator completion and for a broken
        # connection with a socket error from MUTED_SOCKET_ERRORS.
        # Idempotent, because a second call must not count a disconnect twice.
        global LOG_CLIENT_ACTIVE

        if self.closed:
            return
        self.closed = True

        LOG_CLIENT_ACTIVE = False
        if RUN["active"]:
            # The run is still active, so the client disconnected prematurely.
            LOG_STATS["disconnects"] += 1


@app.route("/log.csv")
async def log_csv(request):
    global LOG_CLIENT_ACTIVE

    # There is no await between checking and setting the flag, so this is atomic
    # under asyncio on one core.
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
```

This implementation deliberately has one consumer. A second client gets status
`409`; two consumers may never alternately retrieve records from the same ring.

Because `aclose()` is not called on a `OSError` outside
`MUTED_SOCKET_ERRORS`, `start_run()` must always set `LOG_CLIENT_ACTIVE` to `False`
. Otherwise, one unusual socket error can permanently lock out any subsequent logging client
with a `409`.

### Disconnect semantics

A disconnect:

1. **does not** stop the measurement run;
2. increases `disconnects`;
3. must release the single-client lock again;
4. should never leave the real-time logging task or motor control waiting.

Points 2 and 3 happen in `LogStream.aclose()`. Microdot calls that method after
a normal end of the iterator and on a lost connection with a
socket error from `MUTED_SOCKET_ERRORS`. Because both paths use the same method
, `aclose()` is made idempotent and `disconnects` only counts
as long as the run is still active: if the stream ends because the run has stopped and the
ring is empty, then this is not a disconnect.

Note that Microdot will not notice a dropped client until the next `awrite()`.
If `__anext__()` waits for an empty ring, the disconnect will only become visible at the next
record. That is not a problem for the administration, but
it does mean that `disconnects` may lag slightly.

Because `take_record()` only deletes complete lines, a
reconnect never starts with half a CSV line. A record that had already been transferred to the HTTP stack
at the time of disconnect may be completely lost; the
next `seq` makes that visible.

## Recording on PC

### Start and stop run

A new run must be explicitly started by the application. The exact
Microdot control route can be chosen together with the main web server, but the
semantics are fixed:

```text
start run  -> nieuwe run_id, ring leeg, tellers nul, logger actief
GET log.csv -> connect consumer only; does not change run_id
disconnect -> run blijft actief
stop run   -> no new records; remaining ring contents may still be read
```

For an easy initial implementation, start/stop buttons in
can call existing web interface `start_run()` and `stop_run()`. A
commandline endpoint can be added later if its exact Microdot API
has been captured.

### Include CSV

When the run is active:

```bash
curl --fail --no-buffer \
  http://192.168.4.1/log.csv \
  -o "run_$(date +%F_%H%M%S).csv"
```

Watch live and save at the same time:

```bash
curl --fail --no-buffer http://192.168.4.1/log.csv \
  | tee "run_$(date +%F_%H%M%S).csv"
```

Just stop the **HTTP client** with `Ctrl-C`. That is not the same as
`stop_run()`: the Pico continues to log the active run until the run is explicitly stopped
or the buffer fills up.

If the connection is lost, `curl` is restarted and a new
file is created. `run_id`, `boot_id` and `seq` make visible which files belong together
and where records are missing. Each connection starts with its own header
and, thanks to the record awareness, always with a complete following CSV line.

Automatic reconnection can be done with a PC script. Have each connection write
to a new file; This way, each fragment remains readable independently
and files can be merged later on `run_id` + `seq`.

## Event log on flash

In addition to the measurement stream, save a small event log with only important
events:

- start and end of a run;
- state changes;
- result of the LDR scan;
- grab result;
- emergency stop and errors;
- end values ​​of `ring_dropped`, `log_missed` and `disconnects`.

Only write this log when the motor control is stationary or at another
demonstrably safe moment. Limit the size to a few dozen lines per
trip.

The event log only stores events that are already safe to flash
committed**. There is no guarantee in the event of a sudden loss of power while driving
that the very last RAM state can still be saved. For a guaranteed
"last event at power loss" is additional energy buffering or other hardware
necessary. The event log is therefore a network/diagnostics safety net, not one
transaction log and not a replacement for the measurement stream.

## Alternative 1: raw TCP stream

A TCP stream to `ncat` is only attractive for a bare measurement run without
Microdot or UI.

On PC:

```bash
ncat -lk 9000 > run.csv
```

Cons:

- the Pico must know the IP address of the PC;
- reconnect and header management must be built by yourself;
- partial `send()` operations require a separate pending buffer;
- bytes that have already been removed from the ring may not be replaced behind new data
at `EAGAIN`, because then the order of the CSV stream changes.

A correct sender therefore stores a partially sent chunk separately:

```python
import errno

_WOULD_BLOCK = (errno.EAGAIN, errno.EWOULDBLOCK)


class NetLog:
    def __init__(self, host, port=9000):
        self.addr = (host, port)
        self.sock = None
        self.pending = None
        self.offset = 0

    def service(self):
        if self.sock is None:
            return

        if self.pending is None:
            self.pending = LOG.take_record()
            self.offset = 0
            if self.pending is None:
                return

        try:
            n = self.sock.send(self.pending[self.offset:])
            self.offset += n
            if self.offset >= len(self.pending):
                self.pending = None
                self.offset = 0
        except OSError as exc:
            if exc.args[0] in _WOULD_BLOCK:
                return                       # connection is not broken

            self.sock.close()
            self.sock = None

            # The partially transmitted record MUST be discarded. Resuming at
            # self.offset after reconnect would start the new connection with
            # the tail of an old row: exactly the partial CSV row that the
            # record-aware ring is intended to prevent.
            self.pending = None
            self.offset = 0
```

This class does not itself reconnect; `service()` does nothing as long as `self.sock` is
`None`. Reconnect logic, header resending, and keeping
of a `disconnects` counter still need to be built around it.

Here too, the socket must be non-blocking. A blocking WiFi send can take
long enough to exhaust the motor control's 60 ms FIFO runway.
Please note that `self.pending[self.offset:]` can allocate another temporary object
.

## Alternative 2: websocket

A web socket is suitable for interactive control and UI telemetry, but
is not the preferred route for measurement logging.

Disadvantages for logging:

- client code is needed;
- logging and operation become linked more quickly;
- a browser often stores data in RAM until a download button is pressed;
- in case of a crash or closed tab, the entire ride may be lost.

Therefore, use the existing web socket for commands and approximately 2-5 Hz
telemetry, and the HTTP route for the CSV measurement stream.

## Validation on real hardware

Before commissioning, perform at least the following tests:

1. Run motor control, gyro, ultrasonic, web socket and logging at the same time.
2. Check with a logic analyzer that logging does not indicate any visible engine pauses.
3. Stop `curl` in the middle of a ride and check that the control and the run
keep going.
4. Reconnect and check that the new file starts with a header and
then a **complete** CSV line, never a line fragment.
5. Fill the ring and check that `ring_dropped` rises without blocking.
6. Force a multi-period logger delay and check
`log_missed` and `seq` jump forward without catch-up burst.
7. Check start/stop semantics: new run = empty ring + new `run_id`;
reconnect = same `run_id`.
8. Test the handling of a second concurrent logging client (`409`).
9. Measure heap usage and garbage collection pauses during a long drive at 10 Hz.
10. Repeat a short performance test at 50 Hz for the control tuning profile and measure
the actual line length; check whether the ring space is still at the
desired runway fits.
11. Check the coherence of `gyro_sid`, `gyro_t_ms`, `us_sid` and `ldr_sid`.
12. Before multicore use, please check that the snapshot transfer is not mixed
can produce old/new field combinations.
13. Verify that `LogStream.aclose()` is actually called on a
normal stream end and in the event of a hard disconnect, that `LOG_CLIENT_ACTIVE`
then `False` again and that a subsequent client will therefore not receive `409`.
14. Measure the call rate of `Move.service()` while logging is active and
check that it remains above 100 Hz.
15. Check that an ultrasonic measurement without a valid value shows an empty `us_cm` field
and that the PC analysis reads this as missing and not as 0 cm.
16. When stationary, remove the power supply and check which one is already committed
flash events are preserved; don't claim the last RAM event with that
is guaranteed.

## Summary

| Use | Approach |
|---|---|
| Normal measurement run | HTTP/CSV stream to `curl`, standard 10 Hz |
| Control tuning | Temporary 50 Hz log profile after hardware validation; enlarge ring |
| Controls and screen telemetry | Existing websocket |
| Short network outage | Record-aware RAM ring (approx. 5 s at 10 Hz), run remains active |
| Data loss diagnosis | `seq`, `ring_dropped`, `log_missed`, `disconnects` |
| Network/diagnostics safety net | Small event log on flash for already committed events |
| Bare performance test without web server | Possibly raw TCP stream |

The HTTP stream remains the default solution. The most important design rule remains
that logging should never hold up the motor or control loop. The record-aware
prevents half CSV lines after reconnects, missed log deadlines are skipped
instead of caught up, and a run is deliberately decoupled from the lifetime of
an HTTP connection.

`seq`, `ring_dropped`, `log_missed` and `disconnects` make different shapes
of data loss is separately visible. Signed wheel positions and explicit
sensor IDs/timestamps also make the logs more useful for later
control and trajectory analysis.
