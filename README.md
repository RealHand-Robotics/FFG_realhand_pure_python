# RealHand Pure Python Teleop

This is a standalone, GitHub-ready RealHand teleoperation project written in
pure Python. It does not depend on ROS; it retargets RealForce or RealMCG data
and controls L6, L20, and G20 hands over SocketCAN through the official
RealHand Python SDK.

## Contents

- `realhand_pure_python/`: Pure-Python package for reading RealForce/RealMCG
  data, retargeting, loading calibration data, parsing URDF/config files, and
  converting SDK commands.
- `test_glove_teleop.py`: Main entry point. Supports serial-port scanning,
  RealForce calibration, serial teleoperation, MCG teleoperation, and CAN
  output through `--sdk-send`.
- `control_l20_sdk.py`: SDK helper for inspecting, opening, and sending L20/G20
  poses directly.
- `dump_serial.py`: Raw USB serial-data debugging script.
- `REALFORCE_L6_TELEOP_README.md`: Calibration and runtime instructions for
  L6/L20.
- `environment.yml` / `environment-gloveTeleop.yml`: Verified `gloveTeleop`
  Conda environments.

## Installing the gloveTeleop environment

Run the following commands from the repository root. We recommend creating a
Conda environment named `gloveTeleop`:

```bash
conda env create -f environment-gloveTeleop.yml
conda activate gloveTeleop
python -m pip install -e .
```

If the local machine already has a `gloveTeleop` environment, update its
dependencies with:

```bash
conda env update -n gloveTeleop -f environment-gloveTeleop.yml --prune
conda activate gloveTeleop
python -m pip install -e .
```

Verify the environment and entry script:

```bash
python -c "import realhand_pure_python; print('realhand_pure_python ok')"
python test_glove_teleop.py --help
python test_glove_teleop.py --scan
```

If `conda activate gloveTeleop` is not active in the current shell, any Python
command can instead be run as:

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py --help
```

`environment-gloveTeleop.yml` installs the official RealHand Python SDK. A
connection to the SDK and CAN hardware is required only when physically sending
CAN commands with `--sdk-send` or `control_l20_sdk.py --send`.

You can also use a venv:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e .
```

## Quick check

Run from this directory:

```bash
python test_glove_teleop.py --help
python test_glove_teleop.py --scan
```

Default dual-glove serial ports:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

When the environment is not activated:

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py --scan
```

## USB and CAN

USB permissions:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

Bring up `can0` / `can1` at 1 Mbps:

```bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

ip -details link show can0
ip -details link show can1
```

The expected output includes `UP` and `can state ERROR-ACTIVE`.

## L6 calibration

```bash
python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l6 \
  --right-model l6 \
  --calibration-output realhand_pure_python/config/calibration_l6_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

Follow the prompts to assume the `original`, `opose`, and `fist` poses in order.

## Running L6

```bash
python test_glove_teleop.py \
  --serial \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l6 \
  --right-model l6 \
  --calibration-path realhand_pure_python/config/calibration_l6_dual.yml \
  --no-load-sample-calibration \
  --sdk-send \
  --left-sdk-model l6 \
  --right-sdk-model l6 \
  --left-can can0 \
  --right-can can1 \
  --send-hz 30 \
  --seconds 999999
```

## L20 calibration

```bash
python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-output realhand_pure_python/config/calibration_l20_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

## Running L20

```bash
python test_glove_teleop.py \
  --serial \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-path realhand_pure_python/config/calibration_l20_dual.yml \
  --no-load-sample-calibration \
  --sdk-send \
  --left-sdk-model l20 \
  --right-sdk-model l20 \
  --left-can can0 \
  --right-can can1 \
  --send-hz 30 \
  --seconds 999999
```

For G20 hardware, set both the retargeting and SDK models to `g20`.

## SDK check

Inspect the pose that would be sent without sending CAN commands:

```bash
python control_l20_sdk.py --action open
```

Connect to the SDK and read its state:

```bash
python control_l20_sdk.py --action check --send --read-state
```

Send the open-hand pose:

```bash
python control_l20_sdk.py --action open --send
```

## Notes

- Calibration files are tightly coupled to the gloves, hands, left/right serial
  port assignment, and wearing method. Recalibrate after changing equipment or
  how it is worn.
- `--baudrate 0` is the default and automatically scans for the RealForce baud
  rate.
- If no data arrives after opening a serial port, add `--serial-debug` to view
  diagnostic information.
- Lower-level Python API examples are in `realhand_pure_python/README.md`.
