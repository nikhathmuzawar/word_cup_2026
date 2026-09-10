# AP_MCTRL Library Architecture and Driver Implementation Guide

**Target:** ArduPilot Copter  
**Proposed library:** `AP_MCTRL`  
**Concrete driver:** `AP_MCTRL_Sim`  
**Serial transport base:** `AP_MCTRL_Backend_Serial`  
**Reference device protocol:** MCTRL-SIM ICD v1.0

---

## 1. Purpose

This document explains how a custom ArduPilot library for MCTRL should be structured, how the frontend/backend architecture works, how serial ports are discovered and managed through ArduPilot, how a concrete MCTRL-SIM driver should fit into that architecture, and which classes, relationships, state variables, and methods should appear in the class diagram.

The intended architecture follows the same general frontend/backend pattern used by ArduPilot libraries such as `AP_RangeFinder`.

The high-level structure is:

```text
Vehicle / Copter
      |
      v
+-------------+
|  AP_MCTRL   |   Frontend / manager
+-------------+
      |
      v
+------------------+
| AP_MCTRL_Backend |   Abstract backend interface
+------------------+
      |
      v
+-------------------------+
| AP_MCTRL_Backend_Serial |   Common serial transport support
+-------------------------+
      |
      v
+------------------+
|   AP_MCTRL_Sim   |   Concrete MCTRL-SIM protocol driver
+------------------+
      |
      v
+---------------------+
| AP_HAL::UARTDriver  |
+---------------------+
      |
      v
 MCTRL-SIM device
```

The central design idea is separation of responsibilities:

- `AP_MCTRL` manages the subsystem and exposes a stable interface to the rest of ArduPilot.
- `AP_MCTRL_Backend` defines what all MCTRL backends must implement.
- `AP_MCTRL_Backend_Serial` contains serial-specific setup and UART access.
- `AP_MCTRL_Sim` implements the MCTRL-SIM ICD itself.
- `AP_SerialManager` maps configured serial protocols to actual ArduPilot serial ports.
- `AP_HAL::UARTDriver` is the board-independent UART interface used to read and write bytes.

---

# 2. Proposed Library File Structure

A practical first version of the library could be:

```text
libraries/
└── AP_MCTRL/
    ├── AP_MCTRL.h
    ├── AP_MCTRL.cpp
    ├── AP_MCTRL_Backend.h
    ├── AP_MCTRL_Backend.cpp
    ├── AP_MCTRL_Backend_Serial.h
    ├── AP_MCTRL_Backend_Serial.cpp
    ├── AP_MCTRL_Sim.h
    ├── AP_MCTRL_Sim.cpp
    └── AP_MCTRL_config.h
```

Optional files can be introduced later if parameters or multiple instances become more complex:

```text
    ├── AP_MCTRL_Params.h
    └── AP_MCTRL_Params.cpp
```

The first implementation should stay as small as possible while preserving the frontend/backend architecture.

---

# 3. Frontend/Backend Pattern in ArduPilot

ArduPilot commonly separates a subsystem into:

```text
Frontend
   |
   v
Backend interface
   |
   v
Concrete driver
```

The frontend is what the rest of the flight stack interacts with.

The concrete hardware driver is hidden behind a common backend interface.

This makes the external API stable even if multiple physical implementations exist.

Conceptually:

```text
Copter
   |
   v
AP_MCTRL
   |
   +--> MCTRL-SIM serial backend
   |
   +--> future MCTRL CAN backend
   |
   +--> future hardware-specific backend
```

The vehicle should therefore not need to know that `AP_MCTRL_Sim` exists.

It only needs to know about `AP_MCTRL`.

---

# 4. `AP_MCTRL.h` — Frontend Declaration

`AP_MCTRL.h` defines the public subsystem interface.

Typical responsibilities:

- define the `AP_MCTRL` class;
- define shared device state;
- define backend/device type enums if needed;
- store backend pointer(s);
- provide `init()`;
- provide `update()`;
- provide accessors for data needed by the rest of ArduPilot.

Conceptual structure:

```cpp
#pragma once

#include "AP_MCTRL_config.h"

class AP_MCTRL_Backend;

class AP_MCTRL {
public:
    AP_MCTRL();

    AP_MCTRL(const AP_MCTRL&) = delete;
    AP_MCTRL& operator=(const AP_MCTRL&) = delete;

    enum class Type : uint8_t {
        NONE = 0,
        SIM  = 1,
    };

    enum class DeviceState : uint8_t {
        BOOT      = 0,
        NORMAL    = 1,
        UNDEFINED = 2,
        LOCKOUT   = 3,
    };

    struct State {
        DeviceState device_state = DeviceState::BOOT;

        uint16_t error_flags = 0;
        uint32_t device_uptime_ms = 0;
        float device_temperature_C = 0.0f;

        uint32_t last_health_response_ms = 0;
        bool healthy = false;
    };

    void init();
    void update();

    const State &get_state() const
    {
        return state;
    }

private:
    State state;

    AP_MCTRL_Backend *driver = nullptr;

    void detect_backend();
};
```

---

# 5. What Belongs in Frontend State

The frontend state should contain information useful outside the concrete driver.

Examples:

```text
device operating state
health condition
error flags
device uptime
latest accepted temperature
latest valid telemetry
time of last valid health response
```

The frontend state should not contain implementation details such as:

```text
RX parser buffer
CRC scratch values
current unlock challenge
parser indexes
TX frame scratch arrays
unlock sequence progress
```

Those belong inside the concrete driver.

A useful rule is:

> If the rest of ArduPilot may need to read the value, it is a candidate for frontend state.  
> If only the MCTRL-SIM protocol implementation needs the value, keep it private in `AP_MCTRL_Sim`.

---

# 6. `AP_MCTRL.cpp` — Frontend Implementation

`AP_MCTRL.cpp` implements subsystem lifecycle management.

The main functions are likely to be:

```cpp
void AP_MCTRL::init();
void AP_MCTRL::update();
```

Conceptually:

```cpp
void AP_MCTRL::init()
{
    detect_backend();
}
```

and:

```cpp
void AP_MCTRL::update()
{
    if (driver != nullptr) {
        driver->update();
    }
}
```

The frontend should not parse bytes or calculate CRCs.

Its job is to:

```text
initialise subsystem
       |
       v
select/create backend
       |
       v
call backend->update()
       |
       v
expose shared state
```

---

# 7. Backend Creation

The frontend owns a pointer of base type:

```cpp
AP_MCTRL_Backend *driver;
```

but the actual object may be:

```cpp
AP_MCTRL_Sim
```

For a first implementation with only one type:

```cpp
void AP_MCTRL::detect_backend()
{
    driver = NEW_NOTHROW AP_MCTRL_Sim(state, 0);
}
```

A future parameter-driven version could do:

```cpp
switch (configured_type) {
case Type::SIM:
    driver = NEW_NOTHROW AP_MCTRL_Sim(state, serial_instance);
    break;

case Type::NONE:
default:
    driver = nullptr;
    break;
}
```

The important design point is that the frontend stores the backend using the base-class type.

This enables polymorphism:

```text
AP_MCTRL_Backend*
       |
       +---- actual object ----> AP_MCTRL_Sim
```

When:

```cpp
driver->update();
```

is executed, virtual dispatch calls:

```cpp
AP_MCTRL_Sim::update();
```

---

# 8. `AP_MCTRL_Backend.h` — Common Backend Contract

This class defines the common API all MCTRL drivers must implement.

Example:

```cpp
#pragma once

#include "AP_MCTRL.h"

class AP_MCTRL_Backend {
public:
    explicit AP_MCTRL_Backend(AP_MCTRL::State &_state)
        : state(_state)
    {}

    virtual ~AP_MCTRL_Backend() = default;

    virtual void update() = 0;

protected:
    AP_MCTRL::State &state;
};
```

The backend stores:

```cpp
AP_MCTRL::State &state;
```

as a reference.

This means the frontend owns the state object, while the backend modifies it.

Relationship:

```text
AP_MCTRL
   |
   | owns
   v
 State
   ^
   |
   | referenced by
   |
AP_MCTRL_Backend
```

The backend should not create a duplicate copy of state.

---

# 9. `AP_MCTRL_Backend.cpp`

This file is for implementation shared by all MCTRL backends.

Initially it may contain very little.

Possible responsibilities:

- base constructor implementation;
- common state helper functions;
- transport-independent validity helpers.

Do not place MCTRL-SIM-specific protocol handling here.

A useful test is:

> Would this code still make sense if a future MCTRL backend used CAN rather than UART?

If the answer is no, it probably belongs in a transport-specific or device-specific class instead.

---

# 10. `AP_MCTRL_Backend_Serial.h` — Serial Transport Base

This class derives from `AP_MCTRL_Backend`.

It adds the functionality common to serial MCTRL devices.

Conceptually:

```cpp
class AP_MCTRL_Backend_Serial : public AP_MCTRL_Backend {
public:
    AP_MCTRL_Backend_Serial(AP_MCTRL::State &_state,
                            uint8_t serial_instance);

protected:
    AP_HAL::UARTDriver *uart = nullptr;

    void init_serial(uint8_t serial_instance);

    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;
    virtual uint32_t rx_bufsize() const;
    virtual uint32_t tx_bufsize() const;
};
```

Its role is:

```text
find UART
configure UART
store UART pointer
provide common serial helpers
```

It should not contain:

```text
MCTRL-SIM CRC rules
MCTRL-SIM frame format
health request parsing
fault beacon parsing
unlock key calculations
```

Those are protocol-specific and belong in `AP_MCTRL_Sim`.

---

# 11. `AP_MCTRL_Backend_Serial.cpp` — Serial Port Acquisition

This file should implement UART discovery through `AP_SerialManager`.

Conceptually:

```cpp
void AP_MCTRL_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_MCTRL,
        serial_instance);

    if (uart != nullptr) {
        uart->begin(
            initial_baudrate(serial_instance),
            rx_bufsize(),
            tx_bufsize());
    }
}
```

This is the same architectural idea used by serial backends such as ArduPilot's RangeFinder serial backend.

---

# 12. How `AP_SerialManager` Fits In

`AP_SerialManager` is responsible for associating a configured protocol and baud rate with an available serial port.

Conceptually, a user configures:

```text
SERIALn_PROTOCOL = MCTRL
SERIALn_BAUD     = 115
```

The exact serial number is board-specific.

Your driver does not hard-code:

```text
SERIAL2
TELEM1
/dev/ttyUSB0
```

Instead it asks the SerialManager:

```cpp
AP::serialmanager().find_serial(
    AP_SerialManager::SerialProtocol_MCTRL,
    instance);
```

SerialManager searches for the configured serial port matching that protocol.

The architecture is:

```text
SERIALn_PROTOCOL
       |
       v
AP_SerialManager
       |
       v
matching UARTState
       |
       v
serial index
       |
       v
AP_HAL
       |
       v
AP_HAL::UARTDriver*
```

---

# 13. What `serial_instance` Means

The `serial_instance` argument does not necessarily mean physical serial port number.

It means:

> Which configured instance of this protocol should be returned?

For example:

```text
SERIAL2_PROTOCOL = MCTRL
SERIAL4_PROTOCOL = MCTRL
```

Then:

```cpp
find_serial(SerialProtocol_MCTRL, 0)
```

returns the first matching MCTRL port.

```cpp
find_serial(SerialProtocol_MCTRL, 1)
```

returns the second matching MCTRL port.

This allows one library to support multiple configured devices without hard-coding physical ports.

---

# 14. How the Physical Port is Obtained

Internally, SerialManager works with records describing configured UARTs.

A simplified path is:

```text
find_serial(protocol, instance)
        |
        v
find_protocol_instance(protocol, instance)
        |
        v
matching UARTState
        |
        v
UARTState.idx
        |
        v
hal.serial(serial_idx)
        |
        v
AP_HAL::UARTDriver*
```

The HAL owns the actual UART object.

Your MCTRL backend receives only a pointer to it.

Therefore:

```text
AP_HAL owns UART
AP_MCTRL_Backend_Serial uses UART
```

The backend does not create or delete the UART.

---

# 15. Why ArduPilot Uses `AP_HAL::UARTDriver`

`AP_HAL::UARTDriver` is the hardware-abstraction interface for serial communication.

The same driver code can therefore operate across supported ArduPilot boards without knowing the MCU-specific UART implementation.

Typical operations include:

```cpp
uart->begin(...);
uart->available();
uart->read();
uart->write(...);
```

Architecture:

```text
AP_MCTRL_Sim
      |
      v
AP_HAL::UARTDriver
      |
      v
board-specific HAL implementation
      |
      v
physical MCU UART
```

The concrete MCTRL protocol driver stays board-independent.

---

# 16. Baud Rate

The ICD defines MCTRL-SIM as:

```text
115200 baud
8N1
no hardware flow control
```

There are two implementation strategies.

## Fixed protocol baud

```cpp
uint32_t AP_MCTRL_Backend_Serial::initial_baudrate(...) const
{
    return 115200;
}
```

## SerialManager-configured baud

```cpp
return AP::serialmanager().find_baudrate(
    AP_SerialManager::SerialProtocol_MCTRL,
    serial_instance);
```

For an ICD-defined protocol, a fixed 115200 baud is the simplest interpretation.

If project requirements require user configurability, SerialManager can provide the configured rate with 115200 as the expected/default setting.

This should be agreed before coding.

---

# 17. `AP_MCTRL_Sim.h` — Concrete Driver Declaration

This is the class that implements the actual MCTRL-SIM protocol.

It derives from:

```text
AP_MCTRL_Backend_Serial
```

Conceptual declaration:

```cpp
class AP_MCTRL_Sim : public AP_MCTRL_Backend_Serial {
public:
    AP_MCTRL_Sim(AP_MCTRL::State &_state,
                 uint8_t serial_instance);

    void update() override;

private:
    void read_uart();
    void parse_rx_buffer();

    bool validate_footer(const uint8_t *frame,
                         uint16_t frame_len) const;

    bool validate_crc(const uint8_t *frame,
                      uint16_t frame_len) const;

    void dispatch_frame(const FrameView &frame);

    void handle_imu_raw(const FrameView &frame);
    void handle_mag_field(const FrameView &frame);
    void handle_power_stat(const FrameView &frame);
    void handle_text_status(const FrameView &frame);
    void handle_device_info(const FrameView &frame);
    void handle_health_rsp(const FrameView &frame);
    void handle_fault_beacon(const FrameView &frame);
    void handle_unlock_ack(const FrameView &frame);

    bool send_frame(uint8_t msg_id,
                    const uint8_t *payload,
                    uint8_t payload_len);

    void send_health_request();
    void send_unlock_key(uint32_t key);
    void send_reset_request();

    void update_health();
    void update_recovery();

    uint32_t calculate_key3(uint32_t challenge) const;

    static uint16_t crc16_ccitt_false(
        const uint8_t *data,
        uint16_t len);
};
```

---

# 18. `AP_MCTRL_Sim.cpp` — Protocol Implementation

This file contains nearly all MCTRL-SIM ICD logic:

```text
UART receive
RX buffering
frame synchronization
LEN validation
footer validation
CRC validation
message dispatch
payload decoding
health request generation
health response matching
fault beacon processing
unlock sequence generation
lockout handling
TX frame construction
```

The top-level `update()` should remain small:

```cpp
void AP_MCTRL_Sim::update()
{
    read_uart();
    parse_rx_buffer();

    update_health();
    update_recovery();
}
```

This is preferable to putting all protocol logic directly into `update()`.

---

# 19. Persistent RX Buffer

The protocol allows:

```text
partial frames
multiple frames in one read
junk between frames
attachment to the stream mid-frame
corrupted CRC
```

The concrete driver therefore requires persistent buffering.

Example:

```cpp
static constexpr uint16_t RX_BUFFER_SIZE = 256;

uint8_t _rx_buffer[RX_BUFFER_SIZE];
uint16_t _rx_len = 0;
```

Flow:

```text
UART
 |
 v
read_uart()
 |
 v
_rx_buffer
 |
 v
parse_rx_buffer()
```

The exact buffer size can later be tuned.

---

# 20. Receive Parser Design

Recommended parser:

```text
Find 0xA5 0x5A
      |
      v
Enough header bytes?
      |
      +-- no --> wait
      |
      v
Read LEN
      |
      v
LEN <= 64?
      |
      +-- no --> discard one byte, rescan
      |
      v
frame_len = LEN + 9
      |
      v
Complete frame available?
      |
      +-- no --> wait
      |
      v
Validate footer
      |
      +-- invalid --> discard one byte, rescan
      |
      v
Validate CRC
      |
      +-- invalid --> discard one byte, rescan
      |
      v
Create FrameView
      |
      v
Dispatch
      |
      v
Consume frame
      |
      v
Repeat
```

The parser should never assume one UART read equals one complete frame.

---

# 21. `FrameView`

A useful lightweight helper structure is:

```cpp
struct FrameView {
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    const uint8_t *payload;
};
```

This avoids copying the payload into another object.

It simply points to the already validated data in the RX buffer.

---

# 22. Explicit Endian Helpers

Avoid decoding protocol frames by `reinterpret_cast` into packed structs.

Prefer:

```cpp
static uint16_t read_u16_le(const uint8_t *p);
static int16_t  read_i16_le(const uint8_t *p);
static uint32_t read_u32_le(const uint8_t *p);

static void write_u16_le(uint8_t *p, uint16_t value);
static void write_u32_le(uint8_t *p, uint32_t value);
```

Example:

```cpp
const uint32_t t_ms = read_u32_le(&payload[0]);
const int16_t gyro_x = read_i16_le(&payload[4]);
```

This makes the code:

- explicit;
- portable;
- easy to compare with ICD offsets;
- independent of compiler padding/alignment.

---

# 23. Message Dispatcher

After frame-level validation:

```cpp
switch (frame.msg_id) {
case MSG_IMU_RAW:
    handle_imu_raw(frame);
    break;

case MSG_MAG_FIELD:
    handle_mag_field(frame);
    break;

case MSG_POWER_STAT:
    handle_power_stat(frame);
    break;

case MSG_TEXT_STATUS:
    handle_text_status(frame);
    break;

case MSG_DEVICE_INFO:
    handle_device_info(frame);
    break;

case MSG_HEALTH_RSP:
    handle_health_rsp(frame);
    break;

case MSG_FAULT_BEACON:
    handle_fault_beacon(frame);
    break;

case MSG_UNLOCK_ACK:
    handle_unlock_ack(frame);
    break;

default:
    // Unknown but structurally valid message: ignore.
    break;
}
```

Unknown CRC-valid IDs should not cause parser desynchronization.

---

# 24. Message-Length Validation

Recommended semantic validation:

```text
IMU_RAW       LEN == 18
MAG_FIELD     LEN == 10
POWER_STAT    LEN == 12
HEALTH_RSP    LEN == 13
FAULT_BEACON  LEN == 8
UNLOCK_ACK    LEN == 3
```

Variable-size messages:

```text
TEXT_STATUS   LEN >= 1
DEVICE_INFO   LEN >= 7
```

This validation belongs in the message handling layer after the frame itself has passed structural validation.

---

# 25. TX Frame Builder

All host-to-device packets use the same frame structure.

Use one generic helper:

```cpp
bool send_frame(uint8_t msg_id,
                const uint8_t *payload,
                uint8_t payload_len);
```

Then wrappers:

```cpp
void send_health_request();
void send_unlock_key(uint32_t key);
void send_reset_request();
```

`send_frame()` is responsible for:

```text
SYNC bytes
MSG_ID
SEQ
LEN
payload
CRC
footer
UART write
```

This avoids duplicated framing logic.

---

# 26. TX Sequence Counters

Because sequence numbers are per `MSG_ID`, keep separate counters for outbound commands:

```cpp
uint8_t _health_req_seq = 0;
uint8_t _unlock_key_seq = 0;
uint8_t _reset_req_seq = 0;
```

The counters naturally wrap because they are `uint8_t`.

Incoming sequence numbers should initially be treated as diagnostic only unless project requirements define stronger semantics.

---

# 27. Health Logic

Recommended private members:

```cpp
uint32_t _health_nonce = 0;
uint32_t _last_health_request_ms = 0;
uint32_t _last_health_response_ms = 0;
bool _health_request_pending = false;
```

A simple nonce generator is enough:

```cpp
_health_nonce++;

if (_health_nonce == 0) {
    _health_nonce = 1;
}
```

The protocol requires a non-zero nonce that should differ per request.

No cryptographic randomness is needed for this purpose.

---

# 28. Periodic Health Requests

A recommended host interval is approximately:

```text
1000 ms
```

because the MCTRL-SIM watchdog is 3000 ms.

Example:

```cpp
void AP_MCTRL_Sim::update_health()
{
    const uint32_t now = AP_HAL::millis();

    if ((now - _last_health_request_ms) >= HEALTH_INTERVAL_MS) {
        send_health_request();
        _last_health_request_ms = now;
    }
}
```

No blocking delays should be used.

---

# 29. Health Response Handling

Processing:

```text
HEALTH_RSP
    |
    v
decode nonce
    |
    v
nonce matches current request?
   / \
 no   yes
 |     |
 v     v
drop  decode health
      update frontend state
```

Example:

```cpp
state.device_state =
    static_cast<AP_MCTRL::DeviceState>(decoded_state);

state.error_flags = decoded_error_flags;
state.device_uptime_ms = decoded_uptime;
state.device_temperature_C = decoded_temp * 0.01f;
state.last_health_response_ms = AP_HAL::millis();
state.healthy = true;

_health_request_pending = false;
```

---

# 30. Device State and Recovery State Must Be Separate

Device state is reported by the device:

```cpp
enum class DeviceState : uint8_t {
    BOOT      = 0,
    NORMAL    = 1,
    UNDEFINED = 2,
    LOCKOUT   = 3,
};
```

Driver recovery state describes what the host is currently doing:

```cpp
enum class RecoveryState : uint8_t {
    IDLE,
    WAIT_BEACON,
    SEND_KEYS,
    WAIT_ACK,
    WAIT_LOCKOUT,
};
```

These represent different concepts.

For example:

```text
DeviceState   = UNDEFINED
RecoveryState = WAIT_ACK
```

is valid while the driver waits for the result of an unlock attempt.

---

# 31. Recovery State Machine

Recommended logic:

```text
+------+
| IDLE |
+--+---+
   |
   | fault detected
   v
+-------------+
| WAIT_BEACON |
+------+------+
       |
       | beacon + lockout_ms == 0
       v
+-----------+
| SEND_KEYS |
+-----+-----+
      |
      | send KEY1, KEY2, KEY3
      v
+----------+
| WAIT_ACK |
+----+-----+
     |
     +---- OK --------------------> IDLE
     |
     +---- BAD_KEY / TIMEOUT -----> WAIT_BEACON
     |
     +---- LOCKED_OUT ------------> WAIT_LOCKOUT

+--------------+
| WAIT_LOCKOUT |
+------+-------+
       |
       | beacon reports lockout_ms == 0
       v
  WAIT_BEACON
```

The recovery mechanism should be event-driven and non-blocking.

---

# 32. Recovery Data Members

Recommended private state:

```cpp
RecoveryState _recovery_state = RecoveryState::IDLE;

uint32_t _challenge = 0;
uint8_t _strikes = 0;
uint16_t _lockout_ms = 0;
uint8_t _fault_reason = 0;
```

Each `FAULT_BEACON` updates these values.

---

# 33. KEY3 Calculation

Protocol-defined operation:

```cpp
uint32_t AP_MCTRL_Sim::calculate_key3(uint32_t challenge) const
{
    const uint32_t rotated =
        (challenge << 1) | (challenge >> 31);

    return rotated ^ 0xA5A5A5A5U;
}
```

The latest `FAULT_BEACON` challenge must be used.

---

# 34. Why Recovery Must Be Non-Blocking

Do not implement:

```cpp
send(KEY1);
delay(...);
send(KEY2);
delay(...);
send(KEY3);

while (!ack_received) {
    // wait
}
```

Instead:

```text
scheduler calls update()
        |
        v
process available work
        |
        v
update state
        |
        v
return
```

Later scheduler calls continue the state machine.

This preserves responsiveness of the rest of the flight stack.

---

# 35. `AP_MCTRL_config.h`

This file should contain compile-time feature switches.

Example:

```cpp
#pragma once

#ifndef AP_MCTRL_ENABLED
#define AP_MCTRL_ENABLED 1
#endif

#ifndef AP_MCTRL_SIM_ENABLED
#define AP_MCTRL_SIM_ENABLED AP_MCTRL_ENABLED
#endif
```

Then:

```cpp
#if AP_MCTRL_ENABLED
...
#endif
```

and:

```cpp
#if AP_MCTRL_SIM_ENABLED
...
#endif
```

can be used around relevant code.

Exact defaults should follow the target branch's build-size conventions.

---

# 36. Optional Parameter Files

If needed, parameters can be separated into:

```text
AP_MCTRL_Params.h
AP_MCTRL_Params.cpp
```

Possible parameters:

```text
MCTRL_TYPE
MCTRL_OPTIONS
```

For multiple instances:

```text
MCTRL1_TYPE
MCTRL2_TYPE
...
```

Do not copy the full complexity of `AP_RangeFinder` unless the requirements justify it.

RangeFinder supports many hardware types and instances, so its parameter model is substantially larger than a first custom MCTRL implementation needs.

---

# 37. Complete Runtime Call Flow

```text
Copter / scheduler
       |
       v
AP_MCTRL::update()
       |
       v
AP_MCTRL_Backend::update()
       |
       | virtual dispatch
       v
AP_MCTRL_Sim::update()
       |
       +-----------------------------+
       |                             |
       v                             v
 read_uart()                  update_health()
       |                             |
       v                             |
   RX buffer                        TX
       |                             |
       v                             |
 parse_rx_buffer()                   |
       |                             |
       v                             |
validate frame                       |
       |                             |
       v                             |
dispatch_frame()                     |
       |                             |
  +----+--------------+--------------+
  |                   |
  v                   v
telemetry         health/recovery
handlers             handlers
                         |
                         v
                  update_recovery()
```

---

# 38. Serial Port Discovery Flow

```text
SERIALn_PROTOCOL = MCTRL
SERIALn_BAUD     = configured/default baud
          |
          v
   AP_SerialManager
          |
          v
find_serial(SerialProtocol_MCTRL, instance)
          |
          v
find_protocol_instance(...)
          |
          v
matching UARTState
          |
          v
UARTState.idx
          |
          v
hal.serial(serial_idx)
          |
          v
AP_HAL::UARTDriver*
          |
          v
AP_MCTRL_Backend_Serial::uart
          |
          v
uart->begin(...)
          |
          v
AP_MCTRL_Sim can read/write bytes
```

The custom library does not create a hardware UART.

The HAL owns UART instances.

The serial manager maps configuration to one of those existing UART objects.

---

# 39. Object Ownership Model

Recommended ownership:

```text
Vehicle / Copter
      |
      | owns/contains
      v
AP_MCTRL
      |
      | owns backend pointer
      v
AP_MCTRL_Backend*
      |
      | actual object
      v
AP_MCTRL_Sim
```

State ownership:

```text
AP_MCTRL
   |
   +--> owns State
           ^
           |
           | reference
           |
AP_MCTRL_Backend
           ^
           |
AP_MCTRL_Sim
```

UART ownership:

```text
AP_HAL
   |
   +--> owns UART object
           ^
           |
           | non-owning pointer
           |
AP_MCTRL_Backend_Serial
```

This distinction is useful in the class diagram.

---

# 40. Classes to Include in the Class Diagram

At minimum, include:

```text
AP_MCTRL
AP_MCTRL_Backend
AP_MCTRL_Backend_Serial
AP_MCTRL_Sim
AP_SerialManager
AP_HAL::UARTDriver
```

Useful supporting types:

```text
AP_MCTRL::State
FrameView
DeviceState
RecoveryState
MessageID
UnlockStatus
FaultReason
```

---

# 41. UML Relationships

## `AP_MCTRL` to `AP_MCTRL_Backend`

Relationship:

```text
ownership / composition
```

Reason:

The frontend owns and manages backend instances.

Conceptually:

```text
AP_MCTRL *-- AP_MCTRL_Backend
```

---

## `AP_MCTRL_Backend_Serial` to `AP_MCTRL_Backend`

Relationship:

```text
inheritance
```

```text
AP_MCTRL_Backend
        ^
        |
AP_MCTRL_Backend_Serial
```

---

## `AP_MCTRL_Sim` to `AP_MCTRL_Backend_Serial`

Relationship:

```text
inheritance
```

```text
AP_MCTRL_Backend_Serial
        ^
        |
AP_MCTRL_Sim
```

---

## `AP_MCTRL_Backend` to `AP_MCTRL::State`

Relationship:

```text
association by reference
```

The backend stores:

```cpp
AP_MCTRL::State &state;
```

but the frontend owns the state.

---

## `AP_MCTRL_Backend_Serial` to `AP_SerialManager`

Relationship:

```text
dependency
```

because the serial backend calls SerialManager to find the configured UART.

It does not own SerialManager.

---

## `AP_MCTRL_Backend_Serial` to `AP_HAL::UARTDriver`

Relationship:

```text
association
```

because it stores:

```cpp
AP_HAL::UARTDriver *uart;
```

The pointer is non-owning.

---

# 42. Suggested Class Diagram

```text
+--------------------------------------------------+
|                    AP_MCTRL                      |
+--------------------------------------------------+
| - state : State                                  |
| - driver : AP_MCTRL_Backend*                     |
+--------------------------------------------------+
| + init()                                         |
| + update()                                       |
| + get_state() : const State&                     |
| - detect_backend()                               |
+---------------------------+----------------------+
                            |
                            | owns
                            v
+--------------------------------------------------+
|              AP_MCTRL_Backend                    |
+--------------------------------------------------+
| # state : AP_MCTRL::State&                       |
+--------------------------------------------------+
| + update() : virtual = 0                         |
+---------------------------^----------------------+
                            |
                            | inheritance
                            |
+--------------------------------------------------+
|          AP_MCTRL_Backend_Serial                 |
+--------------------------------------------------+
| # uart : AP_HAL::UARTDriver*                     |
+--------------------------------------------------+
| # init_serial(instance)                          |
| # initial_baudrate(instance)                     |
| # rx_bufsize()                                   |
| # tx_bufsize()                                   |
+---------------------------^----------------------+
                            |
                            | inheritance
                            |
+--------------------------------------------------+
|                 AP_MCTRL_Sim                     |
+--------------------------------------------------+
| - _rx_buffer[]                                   |
| - _rx_len                                        |
| - _health_req_seq                                |
| - _unlock_key_seq                                |
| - _reset_req_seq                                 |
| - _health_nonce                                  |
| - _last_health_request_ms                        |
| - _last_health_response_ms                       |
| - _health_request_pending                        |
| - _recovery_state                                |
| - _challenge                                     |
| - _strikes                                       |
| - _lockout_ms                                    |
| - _fault_reason                                  |
+--------------------------------------------------+
| + update()                                       |
| - read_uart()                                    |
| - parse_rx_buffer()                              |
| - validate_crc()                                 |
| - validate_footer()                              |
| - dispatch_frame()                               |
| - handle_imu_raw()                               |
| - handle_mag_field()                             |
| - handle_power_stat()                            |
| - handle_text_status()                           |
| - handle_device_info()                          |
| - handle_health_rsp()                            |
| - handle_fault_beacon()                         |
| - handle_unlock_ack()                            |
| - send_frame()                                   |
| - send_health_request()                          |
| - send_unlock_key()                              |
| - send_reset_request()                           |
| - update_health()                                |
| - update_recovery()                              |
| - calculate_key3()                               |
| - crc16_ccitt_false()                            |
+--------------------------------------------------+

AP_MCTRL_Backend_Serial
          |
          | uses
          v
+--------------------------+
|    AP_SerialManager      |
+--------------------------+
| + find_serial()          |
| + find_baudrate()        |
| + find_protocol_instance |
+------------+-------------+
             |
             | returns
             v
+--------------------------+
| AP_HAL::UARTDriver       |
+--------------------------+
| + begin()                |
| + available()            |
| + read()                 |
| + write()                |
+--------------------------+
```

---

# 43. Notes to Place on the Class Diagram

Near `AP_MCTRL`:

```text
Frontend / manager
Owns subsystem state and backend lifecycle
Vehicle code interfaces with this class
```

Near `AP_MCTRL_Backend`:

```text
Abstract common backend interface
Provides access to frontend-owned state
```

Near `AP_MCTRL_Backend_Serial`:

```text
Serial transport abstraction
Obtains UART via AP_SerialManager
Does not implement MCTRL-SIM packet logic
```

Near `AP_MCTRL_Sim`:

```text
Concrete protocol driver
Implements MCTRL-SIM ICD v1.0
Parser, CRC, message handlers, health and recovery
```

Near `AP_SerialManager`:

```text
Maps configured SERIALn_PROTOCOL entries
to a HAL UART
```

Near `UARTDriver`:

```text
HAL-owned board-independent UART interface
MCTRL serial backend keeps a non-owning pointer
```

---

# 44. What Not to Include in the Class Diagram

Do not include temporary implementation details such as:

```text
loop indexes
temporary CRC accumulator
temporary decoded gyro variables
temporary TX byte arrays
local timestamps
```

The class diagram should focus on:

```text
classes
ownership
inheritance
associations
major persistent state
important methods
architectural dependencies
```

---

# 45. Useful Initialisation Sequence Diagram

```text
Copter        AP_MCTRL       AP_MCTRL_Sim     SerialBackend    SerialManager      HAL UART
  |               |               |                |                |                |
  |--- init() --->|               |                |                |                |
  |               |-- create --->|                |                |                |
  |               |               |-- base ctor -->|                |                |
  |               |               |                |-- find_serial->|                |
  |               |               |                |                |-- hal.serial -->|
  |               |               |                |<--------- UART pointer ----------|
  |               |               |                |-- uart->begin ------------------>|
  |               |               |<---------------|                                 |
  |<--------------|               |                |                                 |
```

This diagram makes clear that the MCTRL library does not instantiate the physical UART itself.

---

# 46. Useful Runtime Sequence Diagram

```text
Scheduler       AP_MCTRL       AP_MCTRL_Sim       UART
    |               |                |              |
    |--- update() -->|               |              |
    |               |--- update() -->|              |
    |               |                |-- available->|
    |               |                |<-------------|
    |               |                |-- read() --->|
    |               |                |<-------------|
    |               |                | parse        |
    |               |                | validate     |
    |               |                | dispatch     |
    |               |                | health       |
    |               |                | recovery     |
    |<--------------|<---------------|              |
```

---

# 47. Design Decisions to Freeze Before Coding

The following should be explicitly decided:

1. Library name:
   ```text
   AP_MCTRL
   ```

2. Concrete backend:
   ```text
   AP_MCTRL_Sim
   ```

3. Serial protocol enum:
   ```text
   SerialProtocol_MCTRL
   ```

4. Number of MCTRL instances:
   ```text
   one initially
   or
   multiple
   ```

5. Baud policy:
   ```text
   fixed 115200
   or
   SerialManager-configurable with 115200 default
   ```

6. Parameter model:
   ```text
   minimal frontend parameter
   or
   dedicated AP_MCTRL_Params
   ```

7. Frontend state contents.

8. Which telemetry is retained in `AP_MCTRL::State`.

9. Which telemetry is immediately forwarded elsewhere in ArduPilot.

10. RX sequence-number policy.

11. Health-response timeout reporting policy.

12. Logging/diagnostic counters.

---

# 48. Recommended Coding Order

```text
1. AP_MCTRL_config.h

2. AP_MCTRL.h
   - frontend class
   - State
   - Type enum

3. AP_MCTRL.cpp
   - init()
   - update()
   - backend creation

4. AP_MCTRL_Backend.h/.cpp
   - common abstract backend

5. AP_MCTRL_Backend_Serial.h/.cpp
   - UART pointer
   - SerialManager lookup
   - UART begin

6. Add/verify SerialProtocol_MCTRL in AP_SerialManager

7. Verify port lookup works

8. AP_MCTRL_Sim.h
   - protocol constants
   - enums
   - persistent state
   - method declarations

9. CRC helpers

10. endian helpers

11. RX buffering

12. frame parser

13. ICD parser tests

14. message dispatcher

15. telemetry handlers

16. generic TX frame builder

17. HEALTH_REQ / HEALTH_RSP

18. FAULT_BEACON handling

19. unlock/recovery state machine

20. lockout handling

21. telemetry integration with target ArduPilot subsystems

22. end-to-end testing
```

This order minimizes the number of things being debugged at the same time.

---

# 49. Recommended Mental Model

Think of each layer as answering one question.

## `AP_MCTRL`

```text
What does the rest of ArduPilot know about MCTRL?
```

## `AP_MCTRL_Backend`

```text
What must every MCTRL driver provide?
```

## `AP_MCTRL_Backend_Serial`

```text
How does a serial MCTRL driver obtain and configure a UART?
```

## `AP_MCTRL_Sim`

```text
How does the MCTRL-SIM v1.0 protocol actually work?
```

## `AP_SerialManager`

```text
Which configured ArduPilot serial port belongs to MCTRL?
```

## `AP_HAL::UARTDriver`

```text
How do I physically read and write serial bytes on this board?
```

This separation is the central architectural idea behind the custom library.

---

# 50. End-to-End Architecture

```text
SERIALn_PROTOCOL configured as MCTRL
                |
                v
        AP_SerialManager
                |
                v
      protocol-to-port mapping
                |
                v
    AP_MCTRL_Backend_Serial
                |
                v
        UARTDriver pointer
                |
                v
       AP_MCTRL_Sim::update()
                |
       +--------+---------+
       |                  |
       v                  v
     RX path            TX path
       |                  |
       v                  v
    parser            frame builder
       |                  |
       v                  |
   dispatcher              |
       |                  |
 +-----+------+------+     |
 |            |       |    |
 v            v       v    v
telemetry   health  recovery commands
       |
       v
AP_MCTRL::State / ArduPilot subsystem handoff
```

---

# 51. Current ArduPilot Reference Points

The architecture described here is based on the current ArduPilot frontend/backend and serial-management patterns.

Useful source files to inspect in the ArduPilot source tree are:

```text
libraries/AP_RangeFinder/AP_RangeFinder.h
libraries/AP_RangeFinder/AP_RangeFinder.cpp
libraries/AP_RangeFinder/AP_RangeFinder_Backend.h
libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.h
libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.cpp

libraries/AP_SerialManager/AP_SerialManager.h
libraries/AP_SerialManager/AP_SerialManager.cpp
```

Important details to confirm against the exact ArduPilot branch used by the project:

- the assigned serial protocol enum number;
- frontend construction/ownership pattern;
- parameter naming conventions;
- compile-time feature flags;
- buffer-size conventions;
- scheduler integration;
- telemetry handoff interfaces;
- coding style expected by the target branch.

---

# 52. Recommended Class Diagram Final Scope

For the final class diagram, include:

```text
AP_MCTRL
AP_MCTRL::State
AP_MCTRL_Backend
AP_MCTRL_Backend_Serial
AP_MCTRL_Sim
FrameView
AP_SerialManager
AP_HAL::UARTDriver
```

Show:

```text
AP_MCTRL owns AP_MCTRL_Backend
AP_MCTRL owns State
AP_MCTRL_Backend references State
AP_MCTRL_Backend_Serial inherits AP_MCTRL_Backend
AP_MCTRL_Sim inherits AP_MCTRL_Backend_Serial
AP_MCTRL_Backend_Serial depends on AP_SerialManager
AP_MCTRL_Backend_Serial holds a non-owning UARTDriver pointer
AP_SerialManager returns the matching UARTDriver
```

Also place the main persistent driver state inside `AP_MCTRL_Sim`:

```text
RX buffer
RX length
TX sequence counters
health nonce
health timestamps
health pending flag
recovery state
challenge
strike count
lockout time
fault reason
```

This provides enough information to show both the software structure and the relationship between the custom library and the existing ArduPilot serial architecture.
