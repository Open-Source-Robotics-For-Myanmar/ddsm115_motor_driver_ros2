# DDSM115 Protocol Reference

This document describes the byte-level protocol used by the DDSM115 motor driver as implemented by this package.

Files to reference in the codebase:
- Communicator implementation: [src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp](src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp#L1-L220)
- Communicator header: [src/ddsm115_motor_driver_ros2/include/dsm115_motor_driver_ros2/ddsm115_communicator.hpp](src/ddsm115_motor_driver_ros2/include/dsm115_motor_driver_ros2/ddsm115_communicator.hpp#L1-L200)
- Set-ID helper: [src/ddsm115_motor_driver_ros2/scripts/ddsm115_set_id.py](src/ddsm115_motor_driver_ros2/scripts/ddsm115_set_id.py#L1-L200)

---

## Serial parameters
- Baud: 115200
- Data bits: 8
- Parity: none
- Stop bits: 1
- Flow control: none

These are configured in the communicator and set-id script.

---

## Frame overview

- Typical frame length: 10 bytes.
- General layout (indices 0..9):

  [0] ID
  [1] CMD
  [2] D0
  [3] D1
  [4] D2
  [5] D3
  [6] D4
  [7] D5
  [8] reserved / extra
  [9] CRC (Maxim/Dallas CRC-8 over bytes 0..8)

Notes:
- Some special commands (e.g. Set ID) use a different preamble pattern; see "Set Motor ID" below.

---

## Commands implemented in this driver

1) Drive motor (set wheel RPM)
- CMD byte: `0x64` (kCommandDriveMotor)
- Payload: ID, CMD, RPM_H, RPM_L, 0x00,0x00,0x00,0x00,0x00, CRC
- RPM is sent as a signed 16-bit integer (int16) placed into bytes 2 (high) and 3 (low).
- CRC is computed over the first 9 bytes and stored at byte 9.

2) Switch mode (set control mode)
- CMD byte: `0xA0` (kCommandSwitchMode)
- In the C++ `set_wheel_mode` implementation a 10-byte buffer is written where the final byte holds the `Mode` enum value. The code does not call `maxim_crc8` for this buffer, so the device either expects the mode directly in the last byte or the CRC is unnecessary for this packet type. Test carefully.

3) Set motor ID (special boot/config frame)
- Implemented in `ddsm115_set_id.py`.
- Byte pattern built by the script: `[0xAA, 0x55, 0x53, <new_id>, 0x00,0x00,0x00,0x00,0x00, CRC]`
- The script builds 9 bytes (the first 9 shown above) and appends the CRC as the 10th byte.

---

## CRC (Maxim/Dallas CRC-8)

- Polynomial: 0x8C
- Initial CRC: 0x00
- Compute over bytes 0..8; compare/store at byte 9.
- Implementations in the repository:
  - C++ `maxim_crc8` in the communicator ([.../ddsm115_communicator.cpp](src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp#L160-L220)).
  - Python `maxim_crc8` in `ddsm115_set_id.py`.

---

## Drive response (device → host)

- Expected length: 10 bytes.
- Layout (indices):

  [0] ID (should match the requested wheel ID)
  [1] CMD (device command/echo)
  [2..3] current (signed int16, assembled as `(resp[2] << 8) | resp[3]`)
  [4..5] velocity (signed int16, assembled in code as `(resp[5] << 8) | resp[4]` — note the byte-order swap)
  [6..7] position (unsigned uint16, assembled as `(resp[6] << 8) | resp[7]`)
  [8] reserved / extra
  [9] CRC (Maxim/Dallas CRC-8 over bytes 0..8)

### How the driver converts fields into physical units

- `drive_current` (int16) → `current_A = drive_current * (8.0 / 32767.0)`
  - Range mapping: raw ±32767 → ±8 A.

- `drive_velocity` (int16) → `velocity` (in `DriveResponse`) stores the raw numeric RPM value. The hardware interface then calls:
  - `rpm_to_velocity(rpm) = rpm / 60.0 * (2π)` to get radians/second before applying to joint states.

- `drive_position` (uint16) → `position_deg = drive_position * (360.0 / 32767.0)`
  - This maps raw 0..32767 to 0..360 degrees (device uses 32767 as counts-per-revolution reference).

### Validation checks performed by the code

- Ensures at least 10 bytes were read.
- Verifies `resp[0] == requested_id`.
- Verifies `resp[9] == maxim_crc8(resp, 9)`.
- On failure the returned `DriveResponse.result` is left as `failed` and numeric fields are defaulted.

---

## Decode recipe (step-by-step)

1. Receive 10 bytes into `resp`.
2. Verify `resp[0] == expected_id`.
3. Verify `resp[9] == maxim_crc8(resp, 9)`.
4. Compute raw values:
   - `int16 drive_current = (resp[2] << 8) | resp[3];`
   - `int16 drive_velocity = (resp[5] << 8) | resp[4];`  (note swapped bytes)
   - `uint16 drive_position = (resp[6] << 8) | resp[7];`
5. Convert to units:
   - `current_A = drive_current * (8.0 / 32767.0)`
   - `velocity_rpm = (double) drive_velocity`
   - `velocity_rad_s = velocity_rpm / 60.0 * 2π`
   - `position_deg = drive_position * (360.0 / 32767.0)`

---

## Example frames

Note: CRC bytes are shown as `CRC` placeholders. Use the provided Python snippet below to compute CRC for actual use.

- Drive command: set wheel ID=1, RPM=100 (int16 0x0064)

  Bytes (hex): `01 64 00 64 00 00 00 00 00 CRC`

- Expected response (toy values):

  Bytes (hex): `01 64 00 64 10 00 00 80 00 CRC`

  Interpretation:
  - ID = 1
  - CMD = 0x64
  - current = 0x0064 = 100 → current_A = 100 * (8/32767) ≈ 0.0244 A
  - velocity assembled from bytes [5]<<8 | [4] => 0x0010 = 16 rpm → 1.676 rad/s
  - position = 0x0080 = 128 → position_deg = 128 * (360/32767) ≈ 1.406°

---

## Python helper snippets

Use these snippets to build CRC, encode a drive command, and decode a response.

```python
def maxim_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        inbyte = b
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc

def build_drive_cmd(motor_id: int, rpm: int) -> bytes:
    rpm_val = rpm & 0xFFFF
    cmd = bytearray([motor_id, 0x64, (rpm_val >> 8) & 0xFF, rpm_val & 0xFF,
                     0,0,0,0,0])
    cmd.append(maxim_crc8(cmd))
    return bytes(cmd)

def decode_response(resp: bytes, expected_id: int):
    if len(resp) < 10:
        raise ValueError("short response")
    if resp[0] != expected_id:
        raise ValueError("id mismatch")
    if resp[9] != maxim_crc8(resp[:9]):
        raise ValueError("crc fail")
    drive_current = (resp[2] << 8) | resp[3]
    # handle signed int16
    if drive_current & 0x8000:
        drive_current -= 0x10000
    drive_velocity = (resp[5] << 8) | resp[4]
    if drive_velocity & 0x8000:
        drive_velocity -= 0x10000
    drive_position = (resp[6] << 8) | resp[7]

    current_A = drive_current * (8.0 / 32767.0)
    velocity_rpm = float(drive_velocity)
    velocity_rad_s = velocity_rpm / 60.0 * 2.0 * 3.141592653589793
    position_deg = drive_position * (360.0 / 32767.0)

    return dict(current_A=current_A, velocity_rpm=velocity_rpm,
                velocity_rad_s=velocity_rad_s, position_deg=position_deg)
```

---

## Testing tips

- Use the packaged `ddsm115_set_id.py` to send set-id frames safely: `ros2 run ddsm115_motor_driver_ros2 ddsm115_set_id.py /dev/ttyACM0 1`.
- To test drive commands without the full ROS stack, open the serial port at 115200 and write the bytes built with the Python helper.
- If responses don't match expectations, capture raw bytes (e.g., with `cat` or `screen`) and decode with the Python snippet above.

---
