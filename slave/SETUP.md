# Slave drone setup

## 1. System packages
    sudo apt-get install -y python3-pip libopencv-dev

## 2. Python packages
    pip3 install -r requirements.txt

## 3. Identity (set before running, different per drone)
    export DRONE_ID=slave_1     # or slave_2 / slave_3
    export MASTER_IP=10.42.0.1

## 4. Preflight calibration (run once on field, camera pointing at sky)
    python3 00_preflight_calib.py
    # Creates config/fpn_pattern.npy and updates config/origin_state.json

## 5. Run competition mission
    python3 main_orchestrator_competition.py

## Optional: TF Luna lidar
    export LUNA_PORT=/dev/serial0   # default, or /dev/ttyUSB0
    # pyserial must be installed (already in requirements.txt)
