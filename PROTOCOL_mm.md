# DDSM115 အကြောင်း အင်္ဂလိပ် Protocol ကို မြန်မာဘာသာဖြင့် ရှင်းပြချက်

ဤစာရွက်မှာ DDSM115 မော်တာဒရိုင်ဘာ၏ byte-level Protocol ကို မြန်မာဘာသာဖြင့် ရှင်းပြထားပါသည်။

ကိုးကားရန် ဖိုင်များ -
- Communicator implementation: [src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp](src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp#L1-L220)
- Communicator header: [src/ddsm115_motor_driver_ros2/include/dsm115_motor_driver_ros2/ddsm115_communicator.hpp](src/ddsm115_motor_driver_ros2/include/dsm115_motor_driver_ros2/ddsm115_communicator.hpp#L1-L200)
- Set-ID helper script: [src/ddsm115_motor_driver_ros2/scripts/ddsm115_set_id.py](src/ddsm115_motor_driver_ros2/scripts/ddsm115_set_id.py#L1-L200)

---

## Serial ဆက်သွယ်မှု (Serial) သတ်မှတ်ချက်များ
- Baud: 115200
- Data bits: 8
- Parity: none
- Stop bits: 1
- Flow control: none

ဒါကို communicator နဲ့ set-id script တွေမှာ configuration လုပိထားပါတယ်။

---

## Frame Overview
- Typical Frame Length: 10 bytes
- General layout (index 0..9):

  [0] ID
  [1] CMD
  [2] D0
  [3] D1
  [4] D2
  [5] D3
  [6] D4
  [7] D5
  [8] reserved / extra
  [9] CRC (Maxim/Dallas CRC-8 — bytes 0..8 )

မှတ်ချက် — set ID က တခြားပုံစံအတိုင်းသုံးထားတယ်။ အောက်မှာဖတ်ကြည့်လို့ရတယ်။

---

## Driver ထဲမှ အရေးပါ Command များ

1) Drive motor (RPM သတ်မှတ်)
- CMD: `0x64`
- Payload: ID, CMD, RPM_H, RPM_L, 0x00,0x00,0x00,0x00,0x00, CRC
- RPM ကို signed 16-bit (int16) အဖြစ် bytes 2 (high) နှင့် 3 (low) တွင် ထည့်ပို့သည်။
- CRC ကို bytes 0..8 ပေါ်တွင်တွက်၍ byte 9 တွင်ထားသည်။

2) Switch mode (control mode ပြောင်း)
- CMD: `0xA0`
- C++ `set_wheel_mode` က 10-byte buffer တစ်ခု ရေးထုတ်ပြီး အနောက်ဆုံး byte ကို `Mode` enum တန်ဖိုးထည့်ပေးသည်။ code က CRC ကို မတွက်ဘဲပို့ထားတာကြောင့် device က အဲ့နည်းဖြင့် မျှော်လင့်ထားလိမ့်မည်ဟု စဥ်းစားရမယ်။ ပြန်စစ်ဆေးရန်။

3) Set motor ID (boot/config frame)
- implemented: `ddsm115_set_id.py`
- ပုByte pattern built by the script: `[0xAA, 0x55, 0x53, <new_id>, 0x00,0x00,0x00,0x00,0x00, CRC]`

---

## CRC (Maxim/Dallas CRC-8)
- Polynomial: 0x8C
- Initial CRC: 0x00
- bytes 0..8 ပေါ်တွင်တွက်ပြီး byte 9 နှင့် တွဲစစ်ပါ။
- repository တွင် C++ နှင့် Python implementation ရှိသည်။
 (Implementations in the repository:
  - C++ `maxim_crc8` in the communicator ([.../ddsm115_communicator.cpp](src/ddsm115_motor_driver_ros2/src/ddsm115_communicator.cpp#L160-L220)).
  - Python `maxim_crc8` in `ddsm115_set_id.py`.)

---

## Drive response (device → host) အကြောင်း

- Expected length: 10 bytes
- Layout(format) (indices):

  [0] ID (request လုပ်သော wheel ID နှင့် တူရမည်)
  [1] CMD
  [2..3] current (signed int16). assembled: `(resp[2] << 8) | resp[3]`
  [4..5] velocity (signed int16) — driver code တွင် `(resp[5] << 8) | resp[4]` ပုံစံဖြင့် တူနိုင်သော byte-order swap ရှိသည်။
  [6..7] position (uint16) — `(resp[6] << 8) | resp[7]`
  [8] reserved
  [9] CRC (Maxim CRC-8 over bytes 0..8)

### Driver တွင် unit များသို့ ပြောင်းရွှေ့ပုံ
- `drive_current` (int16) → `current_A = drive_current * (8.0 / 32767.0)`
  - raw ±32767 → ±8 A (Range mapping)

- `drive_velocity` (int16) → `DriveResponse.velocity` မှာ raw RPM တန်ဖိုးကို ထားသည်။ Hardware interface မှာ
  - `rpm_to_velocity(rpm) = rpm / 60.0 * (2π)` ဖြင့် rad/s သို့ ပြောင်းပြီး joint state တွင် အသုံးပြုတယ်။

- `drive_position` (uint16) → `position_deg = drive_position * (360.0 / 32767.0)`
  - raw 0..32767 ကို 0..360° သို့ သတ်မှတ်ထားတယ်။

### Driver မှ စစ်ဆေးပုံ
- at least 10 bytes ရရှိမည်ကို စစ်သည်။
- `resp[0] == requested_id` ကို စစ်သည်။
- `resp[9] == maxim_crc8(resp, 9)` ကို စစ်သည်။
- အမှားဖြစ်ပါက `DriveResponse.result` သည် `failed` ဖြစ်ပြီး အခြား numeric fields မကိုင်တွယ်ပါ။

---

## Decode လုပ်ရန် လမ်းညွှန်

1. `resp` အား 10 bytes ရရှိအောင် လက်ခံပါ။
2. `resp[0] == expected_id` ဟုတ်မဟုတ် စစ်ပါ။
3. `resp[9] == maxim_crc8(resp, 9)` ဟုတ်မဟုတ် စစ်ပါ။
4. raw value များ:
   - `int16 drive_current = (resp[2] << 8) | resp[3];`
   - `int16 drive_velocity = (resp[5] << 8) | resp[4];`  (byte swap သတိ)
   - `uint16 drive_position = (resp[6] << 8) | resp[7];`
5. unit သို့ပြောင်း:
   - `current_A = drive_current * (8.0 / 32767.0)`
   - `velocity_rpm = (double) drive_velocity`
   - `velocity_rad_s = velocity_rpm / 60.0 * 2π`
   - `position_deg = drive_position * (360.0 / 32767.0)`

---

## ဥပမာ frame များ
- Drive command (wheel ID=1, RPM=100):

  `01 64 00 64 00 00 00 00 00 CRC`

- Toy response ဥပမာ:

  `01 64 00 64 10 00 00 80 00 CRC`

  ကောက်နုတ်ချက် —
  - current = 0x0064 = 100 → current_A ≈ 0.0244 A
  - velocity assembled (resp[5]<<8 | resp[4]) = 0x0010 = 16 rpm → ~1.676 rad/s
  - position = 0x0080 = 128 → position ≈ 1.406°

---

## Python helper အပိုင်း
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

## စမ်းသပ်ရန် အကြံပြုချက်
- `ddsm115_set_id.py` ကို အသုံးပြု၍ Set-ID frame ပို့နိုင်သည်။
- Python helper ဖြင့် ဖရိမ်းဆောက်၍ raw serial သို့ ပို့ကာ စမ်းသပ်နိုင်သည်။
