# Panda V1 Ground Control (GC) Users Guide

This is the canonical reference for every line sent to and received from Panda V1 over the primary RS-485 link. If something behaves differently from what is described here, treat it as a bug in the firmware, not a docs gap.

## 1. Transport

| Link | Direction | Baud | Framing | Purpose |
|---|---|---|---|---|
| Primary RS-485 (V1 `Serial2`) | GC ↔ V1 | 460800 | newline-terminated ASCII | commands + all telemetry to GC |
| V2↔V1 crossover — direct TTL (V1 `Serial5` ↔ V2 `Serial6`) | V2 → V1 | 460800 | newline-terminated ASCII | PT CSV forwarded from V2 |
| USB CDC (V1 `Serial`) | PC ↔ V1 | 460800 | debug | developer console only |

The V2↔V1 crossover is a direct TTL UART link (RS-485 transceiver bypassed; V2's bus-2 transceiver was broken in hardware). Cable: V2 pin 24 → V1 pin 21, V1 pin 20 → V2 pin 25, common GND only — do not tie 5V rails. Point-to-point, short-run.

Every packet — command or telemetry — is a single line terminated by `\n` (or `\r`). Packets time out after 100 ms idle (`PACKET_IDLE_MS`) even if no newline is seen, so a dropped delimiter is recoverable.

All PT data GC receives has been forwarded from Panda V2. V1 does not have its own PTs; it passes V2's raw `p…` CSV row through as `v2PtData[]` and derives `v2PtPsiData[]` for the bang-bang controllers.

## 2. Command reference

All commands are sent to V1 on `Serial2`. `<side>` is `L` (LOX) or `F` (Fuel). All numeric fields are ASCII decimal.

### 2.1 Arming

| Command | Effect |
|---|---|
| `a` | Energize ARM relay, de-energize DISARM relay. Sets `gArmed = true`. Required before BB `b…1`, `v…1`. |
| `r` | Energize DISARM, de-energize ARM. Sets `gArmed = false`. **Also**: `forceSafe()` on both BB controllers (press + vent closed, ABORT latch cleared, state → DISABLED), `sh.cancelExecution()`, `sh.setAllChannelsOff()`. |

Response lines: `Arming!`, `Disarming!`, `SEQ_ABORT: Outputs de-energized`, optional `SEQ_READY:<raw>`.

### 2.2 Direct solenoid / sequence

| Command | Effect |
|---|---|
| `S<chHex><state>` | Drive DC channel `chHex` (0–F, 1-indexed) to `state` (0 or 1). **Rejected with `CMD_ERROR: chan <n> owned by BB`** if that channel is claimed by a BB controller (press or vent). |
| `s<chHex><state>.<5-digit-ms>` | Append a step to the pending sequence. |
| `f` | Fire the currently loaded sequence. Returns `SEQ_EXEC_START:count=<n>,raw=<cmd>`. |

### 2.3 Bang-bang configuration (persisted to EEPROM)

All three take effect immediately on the running controller and are saved to EEPROM. Format is strict — any parse failure replies `BB_ERROR:parse` and no state changes.

| Command | Fields |
|---|---|
| `B<side><sp>,<db>,<wait>,<maxOpen>` | setpoint psi, symmetric deadband psi (>0), valve-transition debounce ms (≤60000), slow-press max-open ms (≤60000; 0 = disabled) |
| `V<side><trig>,<autoOn01>` | auto-vent trigger psi, auto-vent enable flag (0/1) |
| `M<side><mdot>,<spMin>,<spMax>,<gain>,<rho>,<on01>` | mass-flow target [kg/s], setpoint low/high bounds [psi], gain [psi per (kg/s error) per 500 ms tick], propellant density [kg/m³; ≤0 disables computation even if `on01=1`], enable flag (0/1) |

Each emits `EVT:<ms>:CFG_PUSH:<side>:<k=v,...>` and rewrites the EEPROM block.

### 2.4 Bang-bang state commands

| Command | Effect | Preconditions |
|---|---|---|
| `b<side>1` | Enter `SUSTAIN`. | `gArmed`, BB currently `DISABLED`. |
| `b<side>0` | Leave `SUSTAIN` → `DISABLED`. Press closed. Vent untouched. | Rejected in `ABORT`. |
| `v<side>1` | **Manual auto-vent**: press closed, vent open, state → `AUTO_VENT`. | `gArmed`, vent DC channel configured, not in `ABORT`. |
| `v<side>0` | Exit `AUTO_VENT` → `DISABLED`. **Refused if pressure > deadband-high** — emits `EVT:…:AV_REJECT_CLOSE`. To override, issue `r` (disarm) instead. | Only valid in `AUTO_VENT`. |
| `x<side>` | **Latched ABORT**: press closed, vent open. State → `ABORT`. **Only `r` (disarm) clears the latch.** | None. Safe from any state. |

If the vent channel is unset in `BoardConfig.hpp`, `v<side>1` and `x<side>` emit `EVT:…:AV_NO_HW`. Abort still closes the press valve; vent simply stays untouched.

## 3. Telemetry reference

V1 emits five kinds of line on `Serial2`:

### 3.1 DAQ rows (20 Hz, best-effort)

Order, every telemetry frame:

```
t<f0>,t<f1>,...,t<f11>\n       # LC + TC voltages, NUM_LC_CHANNELS + NUM_TC_CHANNELS = 12
s<f0>,s<f1>,...,s<f11>\n       # Solenoid current voltages, NUM_DC_CHANNELS = 12
p<f0>,p<f1>,...,p<f15>\n       # PT voltages forwarded from V2, NUM_PT_CHANNELS = 16
P<f0>,P<f1>,...,P<f15>\n       # PT pressures scaled to PSI on V1, NUM_PT_CHANNELS = 16
```

All values are floats with 5 decimal places. The `p…` row is the passthrough of V2's PT scan — the identifier character is repeated once per value, matching V2's `CommsHandler::toCSVRow`. Periodic frames are skipped if the UART lacks room so command handling and safety events never wait behind telemetry.

### 3.2 Bang-bang heartbeat (1 Hz, one per side)

```
BB:<side>:<state>:<press01>:<vent01>:<pressure_psi>\n
```

`<state>` is `OFF` / `SUS` / `AV` / `ABT`. `<pressure_psi>` is the PT sample taken this tick, formatted with 1 decimal place. Both sides always print, even when `DISABLED`.

### 3.3 Audit events (on every BB state change)

```
EVT:<ms_uptime>:<category>:<side>:<detail>\n
```

This is the authoritative log for safety review. Categories:

| Category | Emitted when |
|---|---|
| `CFG_PUSH` | `B`/`V`/`M` command accepted and applied. `detail` is the parsed fields. |
| `BB_ON` | Entered `SUSTAIN`. |
| `BB_OFF` | Left a non-`DISABLED` state into `DISABLED` via operator command. |
| `VALVE` | Press or vent actually actuated. `detail` = `press=N,reason=…` or `vent=N,reason=…`. |
| `AV_ENTER` | Entered `AUTO_VENT` (auto-trigger or manual). |
| `AV_EXIT` | Left `AUTO_VENT` (pressure dropped to ≤ deadband-high, or operator close). |
| `AV_REJECT_CLOSE` | `v<side>0` refused because pressure was still above deadband-high. |
| `AV_NO_HW` | Vent-requiring command refused because vent DC channel is unset. |
| `ABORT_ENTER` | Entered latched `ABORT`. |
| `ABORT_CLEAR` | Latch cleared by disarm. |
| `SANITY_FAIL` | PT reading out of `BB_PRESSURE_MIN_PSI..BB_PRESSURE_MAX_PSI` while armed. Auto-latches `ABORT`. |
| `OWN_CONFLICT` | Operator command rejected because BB is already in a state that forbids it (e.g. re-enable while not DISABLED). |
| `MDOT_ADJ` | Mass-flow correction moved the setpoint. `detail` = `mdot=…,err=…,sp=…`. |
| `MDOT_CLAMP` | Mass-flow nudge saturated at `sp_min`/`sp_max`. |

### 3.4 Command acknowledgements / errors

| Line | Cause |
|---|---|
| `BB_ERROR:short` | Command body too short. |
| `BB_ERROR:bad_side` | `<side>` was not `L` or `F`. |
| `BB_ERROR:bad_arg` | Boolean field was not `0` or `1`. |
| `BB_ERROR:parse` | `sscanf` failure or out-of-range field. |
| `BB_ERROR:not_armed` | `b…1` or `v…1` while `!gArmed`. |
| `CMD_ERROR: chan <n> owned by BB` | Manual `S` command on a BB-owned channel. |

### 3.5 Legacy sequence lines

`SEQ_ABORT`, `SEQ_READY`, `SEQ_EXEC_START`, `SEQ_ERROR: No sequence loaded`, `Firing sequence!`, `Arming!`, `Disarming!`, `Panda Initialized!`. These are pre-existing and unchanged.

## 4. V2 passthrough details

V2 owns the 16 PT channels. Every V2 loop iteration it sends its PT CSV row (`p<f>,p<f>,...`) to V1 on `Serial5`. V1:

1. `parseV2PtPacket()` writes the 16 raw current values into `v2PtData[NUM_PT_CHANNELS]` and scales them into `v2PtPsiData[]`.
2. Bang-bang controllers read their configured index out of `v2PtPsiData[]` on every `update()`.
3. V1 re-encodes the full array into a `p…` CSV row and sends it on `Serial2` alongside its own DAQ.

GC sees V2's PT data as `p…` on the primary link; it never sees the raw secondary link. Latency from V2 PT sample to GC `p…` row is one V1 loop iteration (typically well under a millisecond).

If the secondary link drops, `v2PtData[]` and `v2PtPsiData[]` hold stale values. **This is a known limitation** — there is no staleness detector yet. If you need one, add a timestamp next to the PT arrays and have BB self-safe when it ages past a threshold. File this as a separate follow-up if it's needed for flight.

## 5. Hardware channel map (set in `src/hardware-configs/BoardConfig.hpp`)

| Constant | Meaning | Current value |
|---|---|---|
| `BB_LOX_PT_CH` / `BB_FUEL_PT_CH` | Index into `v2PtPsiData[]` for the primary press-line PT | 0, 1 |
| `BB_LOX_DC_CH` / `BB_FUEL_DC_CH` | 1-indexed press-solenoid DC channel | 4, 7 |
| `BB_LOX_VENT_DC_CH` / `BB_FUEL_VENT_DC_CH` | 1-indexed vent-solenoid DC channel | **UNSET** — you must set these before auto-vent / abort do anything. |
| `BB_*_VENTURI_UP_PT` / `BB_*_VENTURI_DN_PT` | 0-indexed venturi taps for mass-flow correction | **UNSET** — mass-flow loop is a no-op until these are set. |
| `BB_PRESSURE_MIN_PSI` / `BB_PRESSURE_MAX_PSI` | Sanity bounds; outside this range while armed → ABORT | −50 … 4000 |
| `BB_EEPROM_MAGIC` | Bumped on `BBConfig` layout changes; old blobs ignored | `0xBB43` |

## 6. Cold-start behavior

1. `setup()` initializes both RS-485 links, SPI, and scanners.
2. `bindIO()` wires the BB controllers to `bbSetChannel` and `bbEmit`.
3. `forceSafe()` is called on both controllers — press closed, vent closed, ABORT latch cleared, state = DISABLED. This runs **before** `bbLoadEeprom()` so persisted config is restored but state is not.
4. `bbLoadEeprom()` restores setpoints/deadbands/wait/maxOpen/vent config/mdot config. `EVT:CFG_PUSH` is emitted three times per side (core, vent, mdot) as the load applies them.
5. `Panda Initialized!` is sent.

The operator must issue `a` to arm and `b<side>1` to start bang-bang.

## 7. Safety-critical defaults to remember

- **Every BB state change emits an `EVT:`** — GC should log this stream and surface the latest line next to the live state.
- **`ABORT` can only be cleared by disarm.** There is no "unabort" command.
- **Sanity-bound violation auto-latches `ABORT`.** The PT read is taken once per tick at the top of `update()`, so a single bad sample is enough.
- **Mass-flow correction only moves the setpoint during `SUSTAIN`** and only if venturi PTs are wired **and** `density_kgm3 > 0`. It never overrides `sp_min`/`sp_max`.
- **Mass-flow formula**: `m_dot [kg/s] = CdA · √(2 · ρ · ΔP)` with `CdA = BB_VENTURI_CDA_M2` (board constant, currently 3.22e-5 m²), `ρ` = per-side `density_kgm3` from the `M` command, `ΔP = max(0, P_up − P_dn)` converted psi → Pa. Upstream/throat areas are not used separately — they're absorbed into CdA.
- **Manual vent close (`v…0`) refuses to close if pressure is still above deadband-high.** Disarm is the forcing function.
- **Disarm always wins.** It runs `forceSafe()` on both sides unconditionally.
