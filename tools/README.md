# DDSM115 Protocol Tool — Usage

This directory contains a small CLI helper to build and decode DDSM115 protocol frames.

Script: `ddsm115_protocol_tool.py`

Prerequisites
- Python 3

Make executable (optional):

```bash
chmod +x src/ddsm115_motor_driver_ros2/tools/ddsm115_protocol_tool.py
```

Basic usage

- Build a drive command (motor ID 1, RPM 100):

```bash
python3 src/ddsm115_motor_driver_ros2/tools/ddsm115_protocol_tool.py build-drive --id 1 --rpm 100
```

- Build a set-ID frame (assign new ID 5):

```bash
python3 src/ddsm115_motor_driver_ros2/tools/ddsm115_protocol_tool.py build-setid --id 5
```

- Decode a 10-byte response (hex):

```bash
python3 src/ddsm115_motor_driver_ros2/tools/ddsm115_protocol_tool.py decode "01 64 00 64 10 00 00 80 00 3A"
```

Output fields for `decode`:
- `id`, `cmd`: raw bytes
- `raw_current`, `raw_velocity`, `raw_position`: integer raw values from the frame
- `current_A`, `velocity_rpm`, `velocity_rad_s`, `position_deg`: converted physical units
- `crc_ok`: boolean indicating if CRC matched

Notes
- The tool computes and verifies Maxim/Dallas CRC-8 (poly 0x8C) to match the driver.
- The tool follows the same byte-order and field decoding as `ddsm115_communicator.cpp` in this repo.

Next steps (optional):
- Add an entry point to package.xml or setup.py to install the tool.
- Add tests for sample frames.
