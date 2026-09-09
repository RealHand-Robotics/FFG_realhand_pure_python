# RealForce Glove Teleoperation Workflow for L6 / L20

Default connections:

- Left glove: `/dev/ttyUSB0`
- Right glove: `/dev/ttyUSB1`
- Left hand: `can0`
- Right hand: `can1`

Run every command below from the `linkerhand_telop_python` repository root.

## 1. Enable the USB ports

```bash
ls -l /dev/ttyUSB0
ls -l /dev/ttyUSB1

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

## 2. Enable the CAN interfaces

L6 / L20 uses 1 Mbps:

```bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

ip -details link show can0
ip -details link show can1
```

The expected output includes `UP` and `can state ERROR-ACTIVE`.

## 3. Calibrate

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
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
After collection, this file is generated:

```text
realhand_pure_python/config/calibration_l6_dual.yml
```

## 4. Run teleoperation

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
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

## 5. When using L20

L20 does not require Python code changes; simply change the model argument from
`l6` to `l20`.

### 5.1 Calibrating L20

We recommend saving a separate calibration file for L20:

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-output realhand_pure_python/config/calibration_l20_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

After collection, this file is generated:

```text
realhand_pure_python/config/calibration_l20_dual.yml
```

### 5.2 Running L20 teleoperation

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
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

For G20 hardware, change the model in the preceding commands to `g20`:

```text
--left-model g20
--right-model g20
--left-sdk-model g20
--right-sdk-model g20
```
