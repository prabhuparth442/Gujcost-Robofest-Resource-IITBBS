
# Documentation: SpeedyBee ArduPilot "Config Error" & Barometer Fix

---

## 1. Overview and Problem Statement

When running ArduPilot on a SpeedyBee flight controller (such as the F405 V3 or V4) via QGroundControl (QGC), users frequently encounter a boot loop accompanied by the following message:

> **"Config Error: Fix problem then reboot"**

---

Often, the QGC Messages tab will specifically cite a Barometer (Baro) initialization failure (e.g., `Baro: unable to initialise driver`).

**The Root Cause:** Unlike basic drone firmware, ArduPilot operates as a massive real-time operating system. It absolutely requires a properly formatted SD card to allocate memory, load parameters, and execute initialization scripts. If the SD card is missing, corrupted, or has the wrong partition structure (like hidden EFI partitions), the firmware "times out" during boot. This timeout prevents sensors like the Barometer from waking up, triggering the Config Error.

---

## 2. The Core Solution: Correct SD Card Formatting (Linux)

A standard format is not enough. ArduPilot requires a single, clean FAT32 partition with a strict **32KB cluster size**. The following steps detail how to wipe and format the SD card correctly using a Linux terminal.

### Step 2.1: Identify the SD Card

Plug the SD card into your Linux machine. Open a terminal and list your block devices:

```bash
lsblk

```

Look for your SD card (e.g., `sda`, `sdb`, `mmcblk0`). *In this specific scenario, the SD card was identified as `/dev/sda` and had two conflicting partitions (`sda1` and `sda2`).*

### Step 2.2: Unmount Existing Partitions

You cannot format a drive while it is in use. Unmount all partitions associated with the SD card:

```bash
sudo umount /dev/sda1
sudo umount /dev/sda2

```

### Step 2.3: Wipe and Rebuild the Partition Table using `fdisk`

ArduPilot will fail if there are multiple partitions. You must delete them and create a single Windows 95 FAT32 (LBA) partition.

---

Run the `fdisk` utility on the main drive:

```bash
sudo fdisk /dev/sda

```

Inside the `fdisk` prompt, type exactly these keys in order, hitting `Enter` after each:

1. `o` *(Creates a new, empty DOS partition table/disklabel)*
2. `n` *(Creates a new partition)*
3. `p` *(Selects 'primary' partition type)*
4. `1` *(Selects partition number 1)*
5. `Enter` *(Accepts default first sector)*
6. `Enter` *(Accepts default last sector, using the whole drive)*
7. `t` *(Changes the partition type)*
8. `c` *(Sets the hex code to 'W95 FAT32 (LBA)')*
9. `w` *(Writes the changes to the disk and exits `fdisk`)*

### Step 2.4: Format the File System with a 32KB Cluster Size

The partition now exists, but it needs a file system. ArduPilot requires specific cluster sizes to read data fast enough without timing out.

Run the following command:

```bash
sudo mkfs.vfat -F 32 -s 64 -n "SPEEDYBEE" /dev/sda1

```

**Command Breakdown:**

* `-F 32`: Forces the FAT32 format.
* `-s 64`: Sets the sectors per cluster to 64. On a standard drive, this creates the exact **32KB cluster size** ArduPilot needs to run efficiently.
* `-n "SPEEDYBEE"`: Names the volume.
* `/dev/sda1`: Targets the newly created partition.

---

## 3. Hardware Initialization & Final Testing

Formatting the card is only half the fix. The hardware must be powered correctly to clear the error.

1. **Insert the SD Card:** Ensure it clicks securely into the SpeedyBee board.
2. **Power via LiPo First:** Do not just plug in the USB. USB ports often provide "dirty" or low voltage. Plug in the main LiPo battery (XT60) first. This ensures the onboard voltage regulators provide a clean 5V to the Barometer and SD card reader.
3. **Connect USB & QGC:** Plug in the USB to your computer and open QGroundControl. Wait for the initialization tones.

### 3.1 Verifying the Fix

---

If the "Config Error" disappears, the fix is successful. To manually verify the Barometer:

1. Go to the **Analyze** tab in QGC.
2. Open the **MAVLink Console**.
3. Type the command: `status`
4. Look for the `Baro` line. If it reads `Baro: 1 OK`, the sensor is actively communicating.

---

## 4. Secondary Fallback Solutions

If the SD card format and LiPo power do not clear the error, check these ArduPilot parameters via QGC (Vehicle Setup > Parameters):

* **Disable External Barometer:** If ArduPilot is hunting for a GPS-mounted barometer that doesn't exist, it will crash.
* Search `BARO_PROBE_EXT` and set it to `0` (None).
* Ensure `BARO_PRIMARY` is `0`.


* **Disable I/O Co-processor:** SpeedyBee boards are All-In-One and lack a secondary I/O chip.
* Search `BRD_IO_ENABLE` and set it to `0` (Disabled).


* **Clear Zombie Parameters:** If flashing over old firmware, old data causes conflicts.
* Go to Tools (in the Parameters screen) and select **Reset all to firmware's defaults**.


* **Hardware Check:** Inspect the Barometer chip (the silver rectangle with a small hole) with a magnifying glass for stray solder balls bridging the pins.


---

# Documentation: Wireless MAVLink Bridge & SSH Troubleshooting

## 5. Wireless Calibration & Telemetry via Raspberry Pi (MAVLink Bridge)

### 5.1 Overview

To eliminate the need for a USB cable during calibration and flight testing, a Raspberry Pi can act as a wireless bridge. It reads serial data from the SpeedyBee flight controller via UART and broadcasts it over Wi-Fi (UDP) to a laptop running QGroundControl (QGC).

### 5.2 Hardware Wiring

Connect the Raspberry Pi's UART pins to a free UART port on the SpeedyBee (e.g., UART4).

* **RPi GND (Pin 6)** $\rightarrow$ **SpeedyBee GND**
* **RPi TX (Pin 8 / GPIO 14)** $\rightarrow$ **SpeedyBee RX (e.g., RX4)**
* **RPi RX (Pin 10 / GPIO 15)** $\rightarrow$ **SpeedyBee TX (e.g., TX4)**

**CRITICAL WARNING:** Do NOT connect the 5V pin from the SpeedyBee to the Raspberry Pi if the Pi is powered by its own USB-C source. This will cause a ground loop and destroy the hardware.

### 5.3 ArduPilot Parameter Configuration

Before disconnecting the USB, configure ArduPilot to send MAVLink data to the chosen UART port.

1. Connect the SpeedyBee to QGC via USB.
2. Go to **Vehicle Setup (Gear icon) > Parameters**.
3. Search for the `SERIAL` port corresponding to your wiring (e.g., `SERIAL4` if using RX4/TX4).
4. Set **`SERIAL4_PROTOCOL`** to `2` (MAVLink 2).
5. Set **`SERIAL4_BAUD`** to `921` (921600 baud rate).
6. Write the parameters and disconnect the USB.

### 5.4 Raspberry Pi Software Setup (MAVProxy & PEP 668)

Modern Linux systems (like Debian Bookworm on the Pi) restrict global python installations to protect system stability (PEP 668). You must install the MAVProxy routing software inside an isolated Virtual Environment.

1. Install the virtual environment tools:
```bash
sudo apt update
sudo apt install python3-venv python3-full

```


2. Create the project folder and the virtual environment:
```bash
mkdir ~/drone_link
cd ~/drone_link
python3 -m venv venv

```


3. Activate the environment:
```bash
source venv/bin/activate

```


4. Install MAVProxy inside the active environment:
```bash
pip install mavproxy

```



### 5.5 Granting Serial Permissions

The default Raspberry Pi user cannot read serial ports. You must add the user to the `dialout` group.

1. Run the command:
```bash
sudo usermod -a -G dialout $USER

```


2. **Reboot the Raspberry Pi** for the permissions to take effect.

### 5.6 Running the Wireless Bridge

To start broadcasting data, execute MAVProxy on the Pi, directing the output to your laptop's IP address.

1. Find your laptop's IP address (Run `hostname -I | awk '{print $1}'` on Linux, or `ipconfig` on Windows). Example: `10.119.242.128`.
2. Activate the virtual environment on the Pi:
```bash
source ~/drone_link/venv/bin/activate

```


3. Start the bridge:
```bash
mavproxy.py --master=/dev/serial0 --baudrate 921600 --out=udp:10.119.242.128:14550

```



### 5.7 Connecting QGroundControl (Laptop)

1. Open QGC on the laptop.
2. Click the **Q icon > Application Settings > Comm Links**.
3. Click **Add**.
4. Set Name to `Drone_WiFi` (or similar).
5. Set Type to `UDP`.
6. Set Port to `14550`.
7. Click **OK**, select the new link, and click **Connect**.
8. Use the MAVLink Console (`status` command) to verify sensor health wirelessly.

---

## 6. Troubleshooting Raspberry Pi Network & SSH

### 6.1 SSH Connection Refused After Reboot

If attempting to SSH into the Pi via an IPv6 link-local address (e.g., `ssh drone2@fe80::...`) results in `Connection refused` immediately after a reboot:

1. **Wait 60 Seconds:** The SSH daemon (`sshd`) is often the last service to load.
2. **Verify IP / Hostname:** The Pi may have a new IP. Attempt connecting via the mDNS hostname:
```bash
ssh drone2@drone2.local

```


3. **Verify Pi Status:** Check the onboard LEDs. Solid Red means power is good. Flashing Green means the OS is reading the SD card. If there is no green flash, the OS is not booting.

### 6.2 Enabling SSH on a Headless Pi

If the SSH service is completely disabled, you can force it to activate without a monitor.

1. Remove the SD card from the Pi and insert it into a computer.
2. Open the `boot` or `bootfs` partition.
3. Create a blank file named exactly `ssh` (no extensions).
* *Linux Command:* `touch /path/to/boot/ssh`


4. Reinsert the SD card into the Pi and power it on.
5. Once connected via SSH, permanently enable the service:
```bash
sudo systemctl enable ssh
sudo systemctl start ssh

```
---

# Documentation: Network Config & Final Wireless Calibration

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 5.2 (Wiring) & 5.3 (Parameters)**
The physical connection between the Raspberry Pi and the SpeedyBee has been changed. You are no longer using UART4. You are using **UART6** (pads labeled `T6` and `R6`).

* **Updated Wiring (Section 5.2):** * RPi TX (Pin 8) $\rightarrow$ SpeedyBee **R6**
* RPi RX (Pin 10) $\rightarrow$ SpeedyBee **T6**


* **Updated Parameters (Section 5.3):** * Connect via USB one last time. In QGroundControl, search for `SERIAL6`.
* Set **`SERIAL6_PROTOCOL`** to `2` (MAVLink 2).
* Set **`SERIAL6_BAUD`** to `921` (921600).
* *Note:* If a GPS was previously using UART6, you must set `GPS_TYPE = 0` for this port and move the GPS to UART1, UART2, or UART3. Write and Reboot.



---

## 7. Dual Network Setup (Ethernet + Wi-Fi) via Netplan

To configure Wi-Fi on the Raspberry Pi without disconnecting your active Ethernet SSH session, you must configure Netplan to run both simultaneously and make them "optional" so the Pi doesn't freeze on boot if one is missing.

### 7.1 Edit the Netplan Configuration

1. Find your specific Netplan file name:
```bash
ls /etc/netplan/

```


*(Usually named `50-cloud-init.yaml` or `01-netcfg.yaml`)*
2. Open the file in the nano editor:
```bash
sudo nano /etc/netplan/YOUR_FILE_NAME.yaml

```


3. Modify the file to exactly match this structure. **Do not use the Tab key; use the Spacebar for indentation.**
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "YOUR_WIFI_SSID":
          password: "YOUR_WIFI_PASSWORD"

```


4. Save (`Ctrl+O`, `Enter`) and Exit (`Ctrl+X`).

### 7.2 Apply Safely

Do not use `netplan apply`. Use the safety-net command:

```bash
sudo netplan try

```

*Why:* If the new configuration drops your Ethernet connection, `netplan try` will automatically revert to the old settings after 120 seconds, saving you from being locked out.

---

## 8. Troubleshooting Wi-Fi connection (`nmcli`)

If connecting via terminal yields a "No network found" error, the Pi's Wi-Fi radio is likely soft-blocked or out of sync.

### 8.1 Force Unblock and Rescan

Run these commands in order to force the radio on and scan the area:

```bash
sudo nmcli radio wifi on
sudo rfkill unblock wifi
sudo nmcli device wifi rescan

```

Wait 5 seconds, then list available networks to verify your SSID is visible:

```bash
nmcli device wifi list

```

### 8.2 Force Connection (Hidden Flag)

To aggressively force the connection to a specific network (e.g., `Rpi_sucks`), use the `hidden yes` flag, which makes the Pi actively probe for it:

```bash
sudo nmcli device wifi connect "Rpi_sucks" password "YOUR_PASSWORD" hidden yes

```

---

## 9. Launching MAVProxy & QGC Wireless Calibration

With the network active and UART6 correctly wired, you can launch the bridge.

### 9.1 Start the Bridge on the Raspberry Pi

Ensure you are using the virtual environment created in Section 5.4.

```bash
source ~/drone_link/venv/bin/activate
mavproxy.py --master=/dev/serial0 --baudrate 921600 --out=udp:10.119.242.128:14550

```

*(Replace `10.119.242.128` with your laptop's current IP if it has changed).*

### 9.2 Connect QGroundControl

1. Open QGroundControl on the laptop.
2. Go to **Application Settings > Comm Links**.
3. Activate the UDP link on Port `14550` created in Section 5.7.
4. Verify connection by checking for the "Heartbeat" message in the MAVProxy terminal.

### 9.3 Perform Wireless Calibration

Because the drone is untethered from the USB cable, hardware calibration can be done freely.

1. In QGC, navigate to **Vehicle Setup (Gear icon) > Sensors > Accelerometer**.
2. Click **Calibrate**.
3. Follow the on-screen prompts to place the drone in 6 positions (Level, Left Side, Right Side, Nose Down, Nose Up, Upside Down). QGC will confirm when each position is saved.
4. If a compass is attached, repeat this process under the **Compass** tab, rotating the drone smoothly across all axes until the progress bar completes.

---

# Documentation: MAVProxy Dependencies, Power Sequencing, & UART Unlocking

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 3 (Hardware Initialization & Final Testing)**
Previously, powering via USB was permitted for initial configuration. This is now **prohibited**.

* **Updated Power Protocol:** The flight controller (SpeedyBee) **must** be powered by a LiPo battery. A standard USB connection cannot provide the necessary current spikes required by the onboard Barometer and Compass. Attempting to initialize ArduPilot with only USB power while connected to a Raspberry Pi will result in a voltage drop (brownout), instantly triggering the `Baro one` and `Config Error: Fix problem then reboot` loop.
* **The "Double Power" Rule:** 1. Raspberry Pi $\rightarrow$ Powered via its own dedicated USB-C cable.
2. SpeedyBee F405 $\rightarrow$ Powered via LiPo battery (XT60/XT30).

---

---

## 10. Resolving MAVProxy Python Dependencies

When launching MAVProxy in a virtual environment on Python 3.12 or newer, you may encounter a dependency crash:
`ModuleNotFoundError: No module named 'future'`

This occurs because MAVProxy relies on legacy Python compatibility modules that are no longer pre-packaged.

### 10.1 Install Missing Modules

Ensure your virtual environment is active, then install the missing packages:

```bash
pip install future

```

If the bridge continues to fail with module errors, install the core serial and MAVLink libraries:

```bash
pip install pyserial pymavlink

```

---

## 11. Unlocking Hardware UART on Ubuntu (`/dev/serial0` Error)

If you launch MAVProxy and receive the following error:
`[Errno 2] could not open port /dev/serial0: [Errno 2] No such file or directory: '/dev/serial0'`

This means the physical RX/TX pins on the Raspberry Pi are disabled at the BIOS level, or the operating system is reserving them for a serial login console. On Ubuntu distributions, the standard `raspi-config` tool is unavailable, requiring manual configuration of the boot files.

### 11.1 Enable the UART Pins

You must edit the hardware configuration file to activate the pins and disable Bluetooth (which frees up the high-speed UART required for a 921600 baud rate).

1. Open the configuration file (do not leave the virtual environment; `sudo` will bypass it):
```bash
sudo nano /boot/firmware/config.txt

```


*(If this file is empty, use `sudo nano /boot/config.txt` instead).*
2. Scroll to the absolute bottom of the file and append these two lines:
```plaintext
enable_uart=1
dtoverlay=disable-bt

```


3. Save (`Ctrl+O`, `Enter`) and Exit (`Ctrl+X`).

---

### 11.2 Disable the Serial Login Console

Linux attempts to send boot logs to the serial pins by default, which will corrupt the MAVLink data stream.

1. Open the command line boot parameters:
```bash
sudo nano /boot/firmware/cmdline.txt

```


*(Or `/boot/cmdline.txt`).*
2. This file contains exactly **one continuous line of text**. Do not add line breaks.
3. Locate and delete **only** the following string:
`console=serial0,115200` (or `console=ttyAMA0,115200`).
4. **CRITICAL:** Do NOT delete `console=tty1`. Keep all remaining text on a single line.
5. Save and Exit.

### 11.3 Remove ModemManager

The `modemmanager` service actively scans open serial ports looking for 4G modems. If it scans the port connected to your SpeedyBee, it will interrupt the connection and cause a crash.
Remove it permanently:

```bash
sudo apt-get remove modemmanager -y

```

### 11.4 Reboot and Verify

Hardware configuration changes require a full system reboot.

```bash
sudo reboot

```

Wait 60 seconds, reconnect via SSH, and verify the port is now active:

```bash
ls -l /dev/serial0

```

*Expected Output:* A symlink pointing to `ttyAMA0` or `ttyS0`. If this appears, the hardware is unlocked and ready for the MAVProxy bridge.

---

# Documentation: Resolving Power Loops & Serial Heartbeat Failures

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 10 (Power Sequencing)**
The previous "Double Power" rule is dangerously flawed if your Electronic Speed Controller (ESC) or UBEC is already hardwired to the Raspberry Pi's 5V rail.

* **Updated Power Protocol (The "Single Power" Rule):** You must never supply 5V to the Raspberry Pi via USB-C if it is also receiving 5V from the LiPo battery via a UBEC. This creates a voltage conflict and a ground loop, triggering the SpeedyBee's Overcurrent Protection (the board shutting down after 3 seconds).
* **Action Required:** Disconnect the USB-C cable entirely. Plug in the LiPo battery. The battery will power the ESC, which powers the UBEC, which powers the Pi. The battery also powers the SpeedyBee. Use the Wi-Fi network configured in Section 7 to SSH into the Pi.

---

---

## 12. Resolving Overcurrent Protection (3-Second Shutdown)

If the SpeedyBee lights up and shuts down after 2-3 seconds, the Power Management IC (PMIC) is detecting a short circuit or massive voltage conflict.

### 12.1 Isolate the Conflict

1. **Unplug all power immediately.** Do not repeatedly plug the battery in to test it, or you will destroy the voltage regulators.
2. **Inspect Solder Joints:** Zoom in on the `T6`, `R6`, `5V`, and `GND` pads. Ensure no microscopic solder balls or stray wire strands are bridging adjacent pads.
3. **Enforce Single Power:** Ensure the Raspberry Pi is only receiving power from ONE source (either isolated USB-C with no 5V wire connected to the FC, OR the drone's UBEC with USB-C disconnected).

---

## 13. Bypassing `/dev/serial0` (Direct Hardware UART)

If editing the boot configuration files (as detailed in Section 11) fails to generate the `/dev/serial0` symlink, or if you refuse to reboot the system, you must point MAVProxy directly to the hardware UART alias.

### 13.1 Identify the Hardware Port

List all active serial devices:

```bash
ls /dev/ttyA* /dev/ttyS*

```

If `/dev/ttyAMA0` is present, the hardware gates are open. This is the primary high-speed UART on the Raspberry Pi.

### 13.2 Launch MAVProxy via Direct Alias

Modify the launch command to target `ttyAMA0` instead of `serial0`.

```bash
source ~/drone_link/venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:10.119.242.128:14550

```

---

## 14. Troubleshooting "Waiting for Heartbeat" (Link 1 Down)

If MAVProxy successfully opens the port but reports `Waiting for heartbeat` and `link 1 down`, the Raspberry Pi is listening, but the SpeedyBee is not transmitting valid data.

### 14.1 The Wiring Cross-Check

Serial communication strictly requires Transmit (TX) to connect to Receive (RX).

1. Verify Pi Pin 8 (TX) connects to SpeedyBee `R6`.
2. Verify Pi Pin 10 (RX) connects to SpeedyBee `T6`.
3. **The Swap Test:** If wired correctly but failing, physically swap the wires at the SpeedyBee. Board labeling is frequently inverted by manufacturers.

---

### 14.2 Terminate Squatter Services

Background Linux services will steal the serial data before MAVProxy can read it. Execute these commands to permanently kill them:

```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service
sudo apt-get remove modemmanager -y

```

### 14.3 Verify ArduPilot Parameters

The flight controller will remain silent if the software is not configured to broadcast MAVLink on UART6.

1. Connect via USB to QGroundControl.
2. Ensure `SERIAL6_PROTOCOL` = `2` (MAVLink 2).
3. Ensure `SERIAL6_BAUD` = `921` (921600).
4. Ensure `BRD_SER6_RTSCTS` = `0` (Disabled).
5. Write parameters and reboot the flight controller.

### 14.4 The Raw Python Diagnostic Test

To isolate whether the issue is MAVProxy configuration or a dead physical link, use a raw Python script to read the pins directly.

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0', 921600, timeout=1); print(s.read(100))"

```

* **If it prints gibberish (e.g., `b'\xfe\x01\x05...'`):** The hardware link is perfect. The SpeedyBee is transmitting. The issue lies within MAVProxy's configuration.
* **If it prints empty (`b''`):** The hardware link is dead. The SpeedyBee is silent. Check your wiring, ArduPilot parameters, or ensure the SpeedyBee is powered by the LiPo battery.

---

# Documentation: Serial Port I/O Errors and Hardware Silence

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 11 (Unlocking Hardware UART)**
The initial `cmdline.txt` edits and service terminations in Section 11 may not be sufficient on Ubuntu. If the operating system aggressively respawns the serial login console, it causes a hard `Input/output error` (Error 5) on the `/dev/ttyS0` port. The commands to stop the `serial-getty` service must be upgraded to a full system `mask`.

---

## 15. Resolving Input/Output Errors (`ttyS0`)

If running a raw Python serial test on `/dev/ttyS0` yields an `Input/output error`, the Linux kernel is actively using the port for a Serial Login Console, creating a lock that prevents MAVProxy from accessing the pins.

### 15.1 Mask the Squatter Services

You must forcefully prevent the OS from ever starting the login services on these ports. Run these commands:

```bash
sudo systemctl mask serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl stop serial-getty@ttyAMA0.service

```

### 15.2 Clean the Boot Command Line

Ensure the boot configuration is entirely clear of serial console assignments.

1. Open the file:
```bash
sudo nano /boot/firmware/cmdline.txt

```


2. Delete `console=serial0,115200` or `console=ttyS0,115200` if they exist.
3. The remaining text **must** be on a single line. Example of a clean configuration:
`multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IN`
4. Save (`Ctrl+O`, `Enter`) and Exit (`Ctrl+X`).
5. **Reboot the Raspberry Pi** (`sudo reboot`) for the kernel to release the lock.

---

## 16. Resolving Hardware Silence (`b''` Output)

If the port opens successfully but the raw Python test returns `b''` (an empty byte string), the software is working perfectly but the physical hardware link is dead. The SpeedyBee is not transmitting data to the Pi.

### 16.1 The Mandatory Wire Swap

If the port is silent, the Transmit (TX) and Receive (RX) wires are almost certainly backwards. Manufacturer pad labeling is frequently inconsistent.

1. Disconnect the data wires from the SpeedyBee.
2. **Reverse them:** Connect the Pi's TX pin to the pad you were previously using for RX. Connect the Pi's RX pin to the pad you were previously using for TX.

### 16.2 Verify Common Ground

Serial communication relies on voltage differentials. Without a shared ground reference, the data stream becomes unreadable noise or silence.

* Ensure a dedicated wire connects a **GND** pin on the Raspberry Pi directly to a **GND** pad on the SpeedyBee. Do not rely solely on the battery ground path.

### 16.3 Verify Advanced ArduPilot Parameters

If the wiring is physically correct but the port remains silent, ArduPilot may have hidden options enabled that corrupt the signal.

1. Connect the SpeedyBee via USB to QGroundControl.
2. Go to Parameters and verify:
* `SERIAL6_PROTOCOL` = `2` (MAVLink 2)
* `SERIAL6_BAUD` = `921` (921600)
* **`SERIAL6_OPTIONS` = `0**` (Ensures signal inversion and half-duplex modes are completely disabled).


3. Write parameters, safely disconnect, and power cycle the flight controller with the LiPo battery.

### 16.4 Test Alternate UART Aliases

On specific Ubuntu builds, the high-speed UART is assigned a non-standard alias. If `ttyAMA0` and `ttyS0` fail, test the following:

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA1', 921600, timeout=1); print(s.read(100))"

```
---

# Documentation: Resolving Kernel Locks and Finalizing UART Configuration

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 11 & Section 15 (Hardware Configuration)**
Stopping background services via `systemctl` is insufficient on certain Ubuntu distributions. If the Linux kernel is explicitly instructed via boot files to use the serial port for a login console, it creates a hard hardware lock, resulting in an `Input/output error` (Error 5). You cannot bypass this with commands; you must edit the boot files and perform a hard reboot.

Additionally, a critical typo was identified in the `config.txt` file instructions. `disable-b` is invalid. It must be exactly `disable-bt`.

---

---

## 17. Understanding Persistent `Input/output error` (Error 5)

If running the Python serial test on `/dev/ttyS0` continually results in an `Input/output error`, the Linux kernel has monopolized the hardware.

* **The Cause:** The OS is actively listening for a user to log in via the TX/RX pins. When MAVProxy or Python attempts to open the port, the OS denies access to prevent system instability.
* **The Symptom (`b''` output):** Conversely, if `/dev/ttyAMA0` returns an empty string (`b''`), it means the high-speed hardware UART is currently assigned to the onboard Bluetooth chip, not your physical GPIO pins.

To resolve both issues simultaneously, the boot configurations must be surgically modified to strip the OS of its control over the serial pins and re-route the high-speed UART to them.

---

## 18. Finalizing Boot Configuration Files

These two files dictate how the Raspberry Pi allocates its hardware the millisecond it receives power.

---

### 18.1 Strip the Serial Console (`cmdline.txt`)

You must ensure the OS does not boot a login console on the serial pins.

1. Open the file:
```bash
sudo nano /boot/firmware/cmdline.txt

```


2. Verify the text is exactly on **one single line**.
3. Ensure the strings `console=serial0,115200` or `console=ttyS0,115200` are completely deleted.
4. *Valid Example Configuration:*
```plaintext
multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IN

```


5. Save and Exit.

### 18.2 Re-route the High-Speed UART (`config.txt`)

The Raspberry Pi has two UARTs. The default "mini-UART" cannot handle the 921600 baud rate required by the SpeedyBee. You must disable Bluetooth to free up the primary "PL011" UART and route it to your pins.

1. Open the file:
```bash
sudo nano /boot/firmware/config.txt

```


2. Scroll to the `[all]` section at the very bottom.
3. Add the following exact commands (ensure there are no typos, such as omitting the 't' in 'bt'):
```plaintext
[all]
enable_uart=1
dtoverlay=disable-bt

```


4. Save and Exit.

---

## 19. The Mandatory Power Reset & Verification

Because the hardware allocation is defined at the BIOS level, a live system cannot apply these changes. A full reboot is mandatory. Furthermore, the SpeedyBee flight controller must be power-cycled to clear its crashed I2C bus (which causes the `Baro` error).

### 19.1 Execution Sequence

1. **Unplug the LiPo battery** from the drone (SpeedyBee powers down).
2. **Reboot the Raspberry Pi:**
```bash
sudo reboot

```


3. Wait 60 seconds for the Pi to reconnect to the network.
4. Re-establish your SSH connection.
5. **Plug the LiPo battery back in.** (The SpeedyBee boots cleanly, with no garbage data coming from the Pi).

### 19.2 The Final Data Test

Verify the connection using the newly assigned high-speed UART (`ttyAMA0`).

1. Activate the virtual environment:
```bash
source ~/drone_link/venv/bin/activate

```


2. Run the diagnostic test:
```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0', 921600, timeout=1); print(s.read(100))"

```



* **Success:** If the output is a string of hexadecimal characters (e.g., `b'\xfd\x01\x05...'`), the hardware link is verified.
* **Failure (`b''`):** If the output is empty, the software is fully unlocked, meaning the failure is strictly physical. Swap the TX and RX wires on the SpeedyBee pads immediately.


---

# Documentation: I2C Bus Conflicts and Compass Integration

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 1 (Overview) & Section 12 (Resolving Overcurrent/Sensor Failures)**
Previously, the `Baro` error was primarily attributed to SD Card formatting, low voltage, or uncalibrated accelerometers. While those remain valid failure points, a **critical new root cause** has been identified: **I2C Bus Jamming**.

If removing the GPS module instantly cures the `Baro` error (but replaces it with a `Compass not healthy` error), the SD card and software configuration are perfectly fine. The external compass and the internal barometer share the same physical data highway (the I2C bus). If the GPS/Compass is miswired, underpowered, or shorting, it crashes the entire I2C bus, taking the internal Barometer offline with it.

---

## 20. Diagnosing the I2C Bus Conflict

The onboard barometer (SPL06 or DPS310) and the external compass (inside the GPS puck) both communicate with the SpeedyBee's CPU using the `SDA` (Data) and `SCL` (Clock) pads.

**The Diagnostic Test:**

1. Unplug the GPS/Compass module completely from the SpeedyBee.
2. Boot the flight controller.
3. If the `Baro` error disappears and is replaced by `Compass not healthy`, you have an absolute confirmation of an I2C hardware conflict.

---

## 21. Resolving I2C Hardware & Wiring Failures

Unlike the UART TX/RX lines used for the MAVProxy bridge, I2C wiring has strict, non-crossing rules and specific power requirements.

### 21.1 The Non-Crossing Rule

Serial (UART) wires must cross (`TX` to `RX`). **I2C wires do NOT cross.**

* Verify GPS `SDA` is wired exactly to SpeedyBee `SDA`.
* Verify GPS `SCL` is wired exactly to SpeedyBee `SCL`.

### 21.2 Inspect for Micro-Shorts

The `SDA` and `SCL` pads on SpeedyBee flight controllers are typically located extremely close together. Use a magnifying glass to inspect the solder joints. A single, microscopic strand of wire bridging `SDA` and `SCL` will instantly crash the entire I2C bus, killing both the compass and the barometer.

### 21.3 Voltage Verification

Compasses require stable power to communicate on the I2C bus. If the compass attempts to draw power and fails, it pulls the data lines to ground (crashing the bus).

* Ensure the GPS/Compass is soldered to a **5V pad**, not a `3.3V` pad (unless explicitly required by your specific GPS module's manual).
* **Power Note:** On many F405 boards, `4V5` pads receive power via USB, but standard `5V` pads *only* receive power when the LiPo battery is connected. If your GPS is on a standard `5V` pad, the compass (and therefore the Baro) will fail if you only power the board via USB.

---

## 22. Firmware and Environmental Baro Fixes

If the wiring is flawless and powered by a LiPo battery, but the conflict persists, the issue lies in the ArduPilot parameters or environmental interference.

### 22.1 Force the Chip ID (SpeedyBee F405 V4)

Newer V4 boards utilize a DPS310 barometer. ArduPilot's auto-detect can occasionally fail to initialize this specific chip.

1. Connect to QGroundControl.
2. Navigate to **Vehicle Setup > Parameters**.
3. Search for `BARO_PROBE_EXT`. Set it to `1` (or ensure the DPS310 bitmask is active).
4. Search for `BARO_OPTIONS` and set to `1`.
5. Reboot the flight controller.

### 22.2 Masking Address Conflicts

If utilizing an older GPS clone (e.g., HMC5883L), its hardcoded I2C address may identical to the onboard Barometer.

1. Search for `COMPASS_TYPEMASK` in parameters.
2. Disable legacy compass drivers one by one to determine if a specific driver is actively fighting the Baro for bus control.

### 22.3 Environmental Light Shielding

Barometers are highly sensitive to ultraviolet and infrared light. Calibrating under direct sunlight or intense bench lighting can cause the sensor to report erratic data, triggering an error lock.

* **Fix:** Place a small piece of open-cell black foam directly over the Barometer chip (the silver rectangular component with a tiny hole). This must be done before flight to prevent wind-pressure anomalies anyway.

---

# Documentation: Resolving ArduPilot I2C Bus Hangs & Boot Loops

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 21 (Resolving I2C Hardware & Wiring Failures)**
The documentation now includes the specific pinout mapping for BZGNSS modules (e.g., BZ-251) utilizing ribbon cables. Ribbon cables are highly susceptible to crossover errors.

* **Standard BZGNSS Pinout vs. SpeedyBee F405 V4:**
* Pin 1 (TX) $\rightarrow$ SpeedyBee R6 (UART 6 RX)
* Pin 2 (RX) $\rightarrow$ SpeedyBee T6 (UART 6 TX)
* Pin 3 (GND) $\rightarrow$ SpeedyBee G (Ground)
* Pin 4 (5V/VCC) $\rightarrow$ SpeedyBee 4V5 or 5V
* Pin 5 (SCL) $\rightarrow$ SpeedyBee SCL
* Pin 6 (SDA) $\rightarrow$ SpeedyBee SDA


* **Micro-Short Warning:** The SCL and SDA pads on the SpeedyBee F405 V4 are directly adjacent. A single, microscopic strand of wire from the ribbon cable bridging these two pads will instantly crash the I2C bus.

---

---

## 23. Diagnosing ArduPilot Boot-Loops (I2C Hangs)

When an I2C bus conflict occurs, ArduPilot will fail to initialize. This failure state is identifiable in the QGroundControl logs by a specific sequence of repeating critical errors.

### 23.1 Recognizing the Boot-Loop Signature

If the QGC log displays the following pattern, the internal hardware bus has crashed:

```plaintext
[Info] Initialising ArduPilot
[Critical] Arm: Gyros not healthy
[Critical] Arm: Baro: not healthy
[Critical] Arm: AHRS: EKF3 not started

```

* **Root Cause:** ArduPilot attempts to initialize. It pings the internal Inertial Measurement Unit (IMU/Gyros) and the Barometer. Because an external device (like a miswired GPS) has shorted the I2C/SPI bus, the internal sensors cannot reply. ArduPilot aborts the boot sequence and restarts, creating an infinite loop.

---

## 24. Resolving Duplicate I2C Address Clashes

If the wiring is perfectly mapped and free of shorts, but the boot-loop persists, the conflict is a hardware address clash. In the I2C protocol, every device must have a unique hexadecimal address.

### 24.1 The BZGNSS / SpeedyBee DPS310 Conflict

Many GPS modules (like the BZ-251) contain an internal DPS310 barometer. The SpeedyBee F405 V4 also utilizes an onboard DPS310 barometer.

* **The Conflict:** Both chips use the exact same I2C address (typically `0x76`). When ArduPilot queries that address, both chips reply simultaneously. The data collides, the bus jams, and the flight controller crashes.

### 24.2 The 2-Wire Hardware Isolation Test

To definitively prove an I2C line conflict without software interference:

1. Keep the GPS module connected to the flight controller.
2. Desolder **only** the `SDA` and `SCL` wires. Leave `5V`, `GND`, `TX`, and `RX` connected.
3. Power the drone.
4. If the log reads `ArduPilot Ready` and `Barometer 1 calibration complete`, the problem is strictly confined to the I2C lines.

### 24.3 Forcing ArduPilot to Ignore the External Barometer

To retain the GPS functionality without crashing the bus, you must instruct ArduPilot to ignore the secondary barometer inside the GPS puck.

1. Connect to QGC and navigate to **Parameters**.
2. Search for **`BARO_PROBE_EXT`**.
3. Set this value to **`0`**. This disables external I2C barometer probing.
4. Reboot the flight controller.

---

## 25. Resolving "Compass Not Healthy" Errors

Once the Barometer address clash is resolved and the drone boots successfully (`ArduPilot Ready`), a `Compass Not Healthy` error will typically remain.

### 25.1 Unlocking Advanced Parameters

ArduPilot 4.5.4 may hide necessary compass parameters.

1. In QGC, click the **QGC Logo > Settings (Gears)**.
2. Under **General > User Experience**, change the **Parameter mode** to **Advanced** (or check "Enable all parameters").

### 25.2 Forcing the Compass Driver

Instead of using a generic typemask, manually enable the compass systems.

1. Navigate to **Parameters**.
2. Search for **`COMPASS_ENABLE`** and set to **`1`**.
3. Search for **`COMPASS_AUTO_ROT`** and set to **`1`** (allows ArduPilot to auto-detect the GPS puck orientation).
4. Reboot the flight controller.

### 25.3 Mandatory Compass Calibration

ArduPilot will strictly flag any compass as "Not Healthy" until it receives a complete, valid data matrix.

1. Go to **Vehicle Setup > Sensors > Compass**.
2. Click **Clear** or **Reset** to purge old, corrupted data from previous I2C crashes.
3. Click **Calibrate** and perform the 6-axis physical rotation sequence. The error will clear immediately upon successful calibration.


---

# Documentation: ArduPilot 4.5+ Parameter Management & Calibration Failures

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 21.2 & 25 (Compass Configuration)**
In modern ArduPilot firmware (V4.5.x and newer), the `COMPASS_TYPEMASK` parameter is deprecated. Do not search for it. Compass management is now handled via individual enable/disable toggles to prevent I2C bus clutter. Furthermore, QGroundControl (QGC) actively hides these parameters by default to simplify the user interface.

---

---

## 26. Unhiding Advanced Parameters in QGC

To resolve advanced hardware conflicts, the QGC interface must be forced to display all backend parameters.

1. Click the **QGC Logo** (top left) $\rightarrow$ **Application Settings** (Gear icon).
2. Under the **General** tab, locate the **User Experience** section.
3. Change **Parameter mode** to **Advanced** (or check "Enable all parameters").
4. Return to the Vehicle Setup menu. The full parameter list will now be visible.

---

## 27. Managing "Ghost" Compasses (I2C Clutter)

ArduPilot supports up to 8 simultaneous compasses. The SpeedyBee F405 lacks an internal compass, meaning the only valid compass is the external GPS puck (Compass 1). If ArduPilot leaves slots 2 through 8 active, it creates "ghost" compasses that poll the empty I2C bus, causing calibration failures.

1. Navigate to **Parameters**.
2. Search for `COMPASS_USE`.
3. Ensure **`COMPASS_USE`** (Compass 1) = `1` (Checked).
4. Set **`COMPASS_USE2`** through **`COMPASS_USE8`** = `0` (Unchecked).
5. Reboot the flight controller.

---

## 28. Resolving Standard PreArm Failsafes

Once the hardware bus is stable (`ArduPilot Ready`), the system will enforce PreArm safety checks. These must be cleared before the drone can arm.

### 28.1 `Critical: PreArm: Battery 1 unhealthy`

* **Cause:** The flight controller is powered solely by USB (5V). ArduPilot expects full flight voltage (e.g., 3S/4S LiPo).
* **Fix:** Plug in the LiPo battery.

### 28.2 `Critical: PreArm: Radio failsafe on`

* **Cause:** No valid RC signal is detected from the receiver.
* **Fix:** Power on the radio transmitter. Ensure it is bound to the receiver (solid green LED). Navigate to the **Radio** tab in QGC and perform a full stick calibration.

### 28.3 `Critical: PreArm: Throttle below failsafe / Check FS_THR_VALUE`

* **Cause:** The throttle stick's lowest PWM value is dropping below the configured failsafe threshold.
* **Fix:** 1. Complete Radio Calibration first.
2. Check the lowest PWM value of your throttle stick (usually around 1000).
3. In Parameters, set **`FS_THR_VALUE`** to a number at least 10 units *below* your lowest stick value, but above the "Radio Off" value (e.g., set to `975`).

---

## 29. Resolving "Cannot start compass thread" (QGC Error)

If `COMPASS_DEV_ID` shows a valid hardware ID (e.g., `855297`) but QGC returns `Cannot start compass thread` when clicking Calibrate, the software is locked out of the hardware initialization process.

### 29.1 Mandatory Prerequisite: Accelerometer Calibration

The compass calibration thread relies on the Extended Kalman Filter (EKF). The EKF cannot start the compass thread if it does not know which way is "down."

* **Fix:** You must complete the 6-axis Accelerometer Calibration *before* attempting the Compass Calibration.

### 29.2 The "Clean Slate" Power Sequence

A dirty boot state will lock the thread. You must execute a hard reset:

1. Close QGC entirely.
2. Unplug USB and LiPo battery.
3. Plug in the **LiPo battery first** (to fully power the GPS/Compass).
4. Plug in the **USB cable**.
5. Open QGC and re-attempt calibration.

### 29.3 Adjusting Calibration Fitness (Magnetic Noise)

If the thread starts but instantly crashes, ambient magnetic interference from the drone frame is too high for the default strictness parameters.

1. Search for parameter **`COMPASS_CAL_FIT`**.
2. Increase the value from the default (`16` or `20`) to **`32`** or **`48`**. This relaxes the algorithmic strictness.

### 29.4 Bypass QGC GUI (MAVLink Console Fallback)

If the QGC user interface is bugged, you can force the calibration thread to spawn via command line.

1. Go to **Analyze Tools $\rightarrow$ MAVLink Console**.
2. Type `magcal start` and press Enter.
3. If the command is accepted, manually rotate the drone through all axes even if no progress bar is visible.

---


# Documentation: Resolving QGC Thread Errors & Pre-Arm Calibration Flags

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 27 (Managing Ghost Compasses) & Section 29 (Thread Errors)**
If `COMPASS_DEV_ID` displays a valid hardware ID (e.g., `855297`) but QGroundControl (QGC) still returns `Cannot start compass thread`, the root cause is frequently a priority assignment failure or a custom orientation matrix conflict within the QGC user interface, rather than a pure ArduPilot parameter issue.


---

## 30. Resolving UI-Induced Thread Crashes

The QGC calibration interface can send conflicting commands to ArduPilot, causing the calibration thread to abort immediately.

### 30.1 Clear Custom Autopilot Rotations

If the QGC UI has a custom orientation selected, it attempts to apply a mathematical matrix that does not align with the raw EKF data, crashing the thread.

1. Navigate to **Vehicle Setup > Sensors > Compass**.
2. Locate the **Autopilot Rotation** dropdown.
3. Change it from `Custom 1` (or any other custom value) to **`None`** (or `Rotation_None`).
*(Note: This assumes the flight controller is mounted facing forward and perfectly flat).*

### 30.2 Assign Sensor Priority

ArduPilot cannot initiate a calibration thread for a sensor that lacks an assigned priority level.

1. In the Compass calibration screen, locate the active compass (the one with the "Use Compass" box checked).
2. Ensure the priority dropdown is explicitly set to **`Priority 1`**. If it reads `Not Set`, the thread will fail.

### 30.3 Disable Fast Calibration

"Fast Calibration" is designed for massive, unmovable vehicles and relies on in-flight estimation. It will cause desktop calibration threads to fail on standard drones.

1. Ensure the **Fast Calibration** checkbox is **unchecked**.

---

## 31. The MAVLink Console Calibration Override

If the QGC graphical interface remains bugged (`Cannot start compass thread`) despite correct parameters, you must bypass the GUI and force the calibration via command-line MAVLink instructions.

1. Navigate to **Analyze Tools > MAVLink Console**.
2. Type exactly: `magcal start 1` and press **Enter**.
*(Note: The `1` specifies that ArduPilot should only attempt to calibrate the compass assigned to Priority 1, bypassing any ghost compasses).*
3. **Crucial:** The console may not provide immediate text feedback. The text may simply disappear. **Do not wait for a prompt.**
4. Immediately pick up the drone and perform the 6-axis rotation "hand dance" for 60 seconds.
5. To check the status of the background thread, type: `magcal status`.

---

## 32. Handling "Calibration Complete" but Persistent Red Errors

---

If the calibration progress bar completes and turns green (or the log states `MAG0 initial yaw alignment complete`), but the Compass tab remains red and throws a `PreArm: Compass not calibrated` error, the flight controller has not yet transferred the new offset data into active memory.

### 32.1 The Mandatory Hard Reboot

ArduPilot caches the pre-arm failure state. It will not re-evaluate the compass health until the system is completely power-cycled.

1. Do not use the software reboot button.
2. Unplug the **LiPo Battery**.
3. Unplug the **USB Cable**.
4. Wait 10 seconds to allow capacitors to drain.
5. Reconnect power. The red error will clear.

### 32.2 Disabling Auto-Learning Override

If the error persists after a hard reboot, ArduPilot's in-flight learning algorithm may be rejecting the manual ground calibration.

1. Navigate to **Parameters**.
2. Search for **`COMPASS_LEARN`**.
3. Set the value to **`0`** (Disabled). This forces the EKF to strictly utilize the offsets generated during the manual rotation.


---

# Documentation: Radio Integration & Final Pre-Arm Failsafes

## [CRITICAL UPDATE TO PREVIOUS DOCUMENTATION]

**Changes applied to Section 32 (Handling Calibration Errors)**
If the graphical calibration consistently fails or throws ghost errors despite successful log entries, ArduPilot has a built-in fallback mechanism to calibrate the compass during flight.

### 33. The "In-Flight" Calibration Fallback (`COMPASS_LEARN`)

If ground calibration is completely blocked by UI glitches, you can force ArduPilot to learn the magnetic offsets dynamically.

1. Navigate to **Parameters**.
2. Search for **`COMPASS_LEARN`**.
3. Set the value to **`3`**.
4. The Messages tab will display `CompassLearn: Initialised`.
5. **Execution:** Take the drone outside to acquire a 3D GPS lock. Arm the drone in **Stabilize** mode only (do not use Loiter or Auto). Fly in slow circles and figure-eights for 1–2 minutes.
6. Once sufficient data is gathered, the log will output `CompassLearn: finished`, the parameter will reset to `0`, and the compass error will permanently clear.

---

## 34. Resolving Radio Failsafes (FlySky iA6B Integration)

Once the compass is calibrated, ArduPilot will evaluate the radio link. If the log displays `Critical: PreArm: Radio failsafe on`, the flight controller is not receiving a valid control signal from the receiver.

### 34.1 The PWM/Servo Pin Misconception

**Do not connect the horizontal servo pins (CH1 - CH6).** Modern flight controllers do not use legacy PWM (one wire per channel). ArduPilot requires a multiplexed digital signal (i-BUS or SBUS) transmitted over a single wire.

### 34.2 FlySky iA6B i-BUS Wiring

The FlySky iA6B receiver features a dedicated digital output port.

1. Locate the vertical cluster of 3 pins on the far right edge of the receiver. This is labeled **SENS** or **i-BUS**.
2. **Wiring to SpeedyBee F405:**
* **Top Pin (Signal):** Connect to a UART RX pad (e.g., `RX2` or `SBUS`).
* **Middle Pin (Power):** Connect to a `5V` pad.
* **Bottom Pin (Ground):** Connect to a `G` (Ground) pad.



---

### 34.3 Transmitter Configuration

The remote control must be instructed to output the digital signal.

1. Turn on the FlySky FS-i6.
2. Hold `OK` to enter the menu.
3. Navigate to **System Setup > RX Setup > i-BUS Setup** (or `PPM Output` on older firmware).
4. Ensure the output is set to **i-BUS** (or turn PPM to `On` if i-BUS is unavailable).

---

## 35. Resolving Throttle Failsafes (`Check FS_THR_VALUE`)

If the log displays `Critical: PreArm: Throttle below failsafe` or `Check FS_THR_VALUE`, ArduPilot's emergency system is triggering because the baseline throttle signal is lower than the configured safety threshold.

### 35.1 Adjusting the Safety Buffer

FlySky receivers typically output a PWM value of exactly `1000` when the throttle stick is at zero. If the ArduPilot failsafe threshold is set to `1000` or higher, resting the stick at zero triggers a panic state (simulating a lost connection).

1. Navigate to **Parameters**.
2. Search for **`FS_THR_VALUE`**.
3. Set the value to **`975`**.
* *Logic:* This creates a 25-point safety buffer. The drone recognizes `1000` as zero throttle, but will only trigger a failsafe if the signal drops to `975` (which only happens if the radio physically powers off or disconnects).



---

## 36. Final Radio Calibration Sequence

Even with perfect wiring and parameters, ArduPilot will refuse to arm until the exact range of the transmitter gimbals has been mapped to the flight controller.

1. Open QGroundControl and navigate to the **Radio** tab (remote control icon).
2. Ensure the transmitter is powered on and bound to the receiver.
3. If the wiring is correct, the channel bars will be red and will move when you manipulate the sticks. (If the bars are grey and motionless, return to Section 34.2 and check your UART wiring).
4. Click **Calibrate**.
5. Move both sticks to their extreme maximum and minimum boundaries (all corners). Flip all assigned switches.
6. Click **Save/Apply**.

---

## 37. Final Arming Checklist

Before issuing the Arm command, verify the resolution of all critical flags:

| Pre-Arm Error Flag | Resolution Status | Required Action |
| --- | --- | --- |
| **Baro: not healthy** | Cleared | I2C conflict resolved. `BARO_PROBE_EXT` = 0. |
| **Compass not calibrated** | Cleared | Calibration saved. Hard reboot performed. |
| **Radio failsafe on** | Cleared | iA6B wired via i-BUS. Sticks calibrated in QGC. |
| **Throttle below failsafe** | Cleared | `FS_THR_VALUE` set to 975. |

When the HUD displays `ArduPilot Ready` without critical flags, pull the throttle stick to the bottom right corner for 3 seconds. The motors will spin.

---

# Documentation: Radio Integration, Flight Modes & In-Flight Calibration

## 38. FlySky FS-i6 & iA6B Receiver i-BUS Setup

Modern flight controllers do not use legacy PWM (individual wires for channels 1-6). You must use a single-wire digital multiplexed protocol. For the FlySky iA6B, this is i-BUS.

### 38.1 Receiver Wiring (i-BUS Port)

Do not connect anything to the horizontal servo pins. Locate the vertical 3-pin cluster on the top-right edge of the receiver labeled **iBus servos** (or **SENS**).

1. **Top Pin (S / Signal):** Connect to `RX2` (or `SBUS`) on the SpeedyBee F405 V4.
2. **Middle Pin (V+ / Power):** Connect to a `5V` pad.
3. **Bottom Pin (G / Ground):** Connect to a `G` pad.

### 38.2 Transmitter Configuration

The remote must be forced to output the digital i-BUS signal.

1. Turn on the FS-i6 and hold **OK** to enter the menu.
2. Go to **System Setup > RX Setup > i-BUS Setup** (or `PPM Output` on older models).
3. Ensure the output is set to **i-BUS**.
4. Hold **Cancel** to save and exit.

---

## 39. Configuring ArduPilot for i-BUS

ArduPilot must be instructed to listen for the RC signal on the specific UART port where the receiver is wired.

1. In QGroundControl, navigate to **Parameters**.
2. If wired to `RX2`, search for **`SERIAL2_PROTOCOL`**. Set it to **`23`** (RCIN).
3. Search for **`SERIAL2_BAUD`**. Set it to **`115`** (115200).
4. Write parameters and reboot the flight controller.

---

## 40. Overriding Persistent Radio & Throttle Failsafes

FlySky transmitters frequently output a baseline value around `1000` when the throttle is at zero. If ArduPilot's failsafe threshold is too high, it traps the system in a loop of `Radio failsafe on` and `Throttle below failsafe` errors, even when the connection is perfect.

### 40.1 Drop the Failsafe Threshold

You must force the software safety floor below the physical limit of the transmitter.

1. Search for **`FS_THR_VALUE`**.
2. Drop the value from the default (`975`) down to **`950`**, **`925`**, or even **`900`**.
3. Write parameters. The `Radio Failsafe Cleared` message should now remain stable.

### 40.2 Resolve `Arm: Throttle too high`

Once the failsafe is cleared, ArduPilot will reject arming if it does not know the exact physical range of your sticks.

1. Go to the **Radio** tab in QGC.
2. Click **Calibrate**.
3. Move both sticks to their absolute physical limits (all corners).
4. Click **Save/Apply**. This maps the exact zero point, clearing the throttle error.

---

## 41. In-Flight Compass Calibration (`COMPASS_LEARN`)

If the QGC calibration thread consistently crashes or throws `PreArm: Compass not calibrated` despite green progress bars, bypass ground calibration entirely using ArduPilot's automatic in-flight learning.

1. Go to **Parameters** and search for **`COMPASS_LEARN`**.
2. Set the value to **`3`**.
3. The log will output `CompassLearn: Initialised`.
4. **Execution:** Ensure the GPS has a 3D lock (solid blue LED). Arm the drone in **Stabilize** mode only. Fly slow circles and figure-eights.
5. The log will output `CompassLearn: finished`. The parameter will automatically reset to `0`, and the compass error is permanently resolved.

---

## 42. Configuring Flight Modes and Arm Switches (FlySky FS-i6)

To control the drone safely, you must map the physical switches on the transmitter to auxiliary channels, and then assign those channels to ArduPilot actions.

### 42.1 Map Switches on the Transmitter

1. Hold **OK** to enter the FS-i6 menu.
2. Go to **Functions Setup > Aux. channels**.
3. Set **Channel 5** to **SwC** (the 3-position switch for flight modes).
4. Set **Channel 6** to **SwD** (the 2-position switch for arming).
5. Hold **Cancel** to save.

### 42.2 Assign Flight Modes in QGC

1. In QGC, go to the **Flight Modes** tab.
2. Set the **Mode Channel** to **Channel 5**.
3. Toggle `SwC` on the remote to see which rows highlight green. Assign them as follows:
* **Switch Up (Pos 1):** `Stabilize` (Manual leveling, required for first flight).
* **Switch Middle (Pos 2):** `AltHold` (Automatically maintains altitude).
* **Switch Down (Pos 3):** `Loiter` (GPS position hold).



### 42.3 Assign the Arm/Disarm Switch

A physical kill switch is mandatory for safety.

1. Go to **Parameters** and search for **`RC6_OPTION`** (which corresponds to Channel 6 / SwD).
2. Set the value to **`41`** (ArmDisarm).
3. Write parameters. Flipping `SwD` up will now instantly arm the drone; flipping it down will instantly disarm it.


---

# Documentation: Arming Configuration, Motor Diagnostics & Logging Failures

## 43. Configuring a Physical Arming Switch

While ArduPilot allows arming via stick commands (Throttle Down + Yaw Right), a dedicated physical switch is significantly safer and faster for emergency disarming.

### 43.1 Transmitter Configuration (FS-i6)

1. Enter the FS-i6 menu (Hold `OK`).
2. Navigate to **Functions Setup > Aux. channels**.
3. Set **Channel 6** to **SwD** (the 2-position switch).
4. Save and Exit.

### 43.2 ArduPilot Parameter Configuration

1. In QGroundControl, navigate to **Parameters**.
2. Search for **`RC6_OPTION`**.
3. Set the value to **`153`** (ArmDisarm).
*(Note: Older firmware used `41`. ArduCopter V4.5+ frequently rejects `41` as an `Invalid channel option`. Use `153` to bypass this error).*
4. Write parameters and reboot.

### 43.3 Verifying Switch Travel (PWM Range)

ArduPilot will strictly ignore the arm command if the switch does not send a high enough PWM value (typically >1800).

1. Go to the **Radio** tab in QGC.
2. Flip `SwD`.
3. Observe the bar for **Channel 6**. It must move from the far left (~1000) to the far right (~2000).
4. **Fix:** If the bar only moves halfway, go to the FS-i6 **Functions Setup > End Points** and increase Channel 6 to `120%` on both the high and low ends.

---

## 44. Resolving Motor Rotation Failures

If the drone arms successfully but one motor stutters, twitches, or fails to spin, you must isolate the hardware from the software configuration.

### 44.1 The QGC Motor Test (Software Bypass)

1. **REMOVE ALL PROPELLERS.**
2. Navigate to **Vehicle Setup > Motors**.
3. Slide the safety switch on the screen to enable testing.
4. Click **Test Motor** for the failing unit.
* **If it spins normally:** The hardware is perfect. The issue is a corrupted ESC calibration or a radio mixing error.
* **If it stutters/fails:** The issue is physical.



### 44.2 Hardware Diagnostics

* **Phase Wire Disconnect:** A twitching motor usually indicates that one of the three wires connecting the motor to the ESC is loose, broken, or suffering from a cold solder joint.
* **Mounting Screw Short:** If the screws securing the motor to the frame are too long, they will pierce the copper stator windings, creating an electrical short that prevents rotation.

### 44.3 DShot Configuration

Modern ESCs should not use legacy PWM or require manual end-point calibration.

1. In Parameters, search for **`MOT_PWM_TYPE`**.
2. Ensure it is set to **`4`** (DShot600).
3. Write parameters and reboot. DShot is a digital protocol that is immune to throttle-range desynchronization.

---

## 45. Resolving "Logging Failed" Pre-Arm Errors

ArduPilot is configured by default to refuse arming if it cannot write flight telemetry to the onboard SD card. If you see `Logging Failed`, the SD card is missing, corrupted, or communicating too slowly.

### 45.1 Hardware formatting (The `FAT32` Rule)

The SpeedyBee F405 V4 cannot read `exFAT` partitions.

* The SD card must be 32GB or smaller (or artificially partitioned to be smaller).
* It must be formatted strictly to **FAT32** with a **32KB cluster size** (See Section 2 of this manual for exact formatting instructions).

### 45.2 Slowing the SPI Bus

If a valid FAT32 card is inserted but the error persists, the flight controller is attempting to write data faster than the SD card can accept it.

1. In Parameters, search for **`BRD_SD_SLOWDOWN`**.
2. Increase the value from `0` to **`2`** or **`5`**.
3. Write parameters and reboot.

### 45.3 Bypassing the Logging Requirement

If you do not require flight logs or the SD card reader is physically broken, you can instruct ArduPilot to ignore the failure and permit arming.

1. **Disable the Log Stream:** Search for **`LOG_BITMASK`** and set it to **`0`**.
2. **Disable the Pre-Arm Check:** Search for **`ARMING_CHECK`**. Click the dropdown and uncheck **Logging Available**. (Alternatively, set to `0` for bench testing only).
3. Write parameters and reboot. The drone will now arm without an SD card.

---

# Documentation: Advanced Motor Diagnostics & Hardware Isolation

## 46. Resolving Symmetrical Throttle Lag (Slow Motor Spin)

If all motors spin, but one accelerates noticeably slower than the others, the issue is typically a desynchronization between the ArduPilot throttle output and the ESC's calibrated range.

### 46.1 ESC Calibration (PWM Only)

If utilizing standard PWM, you must teach the ESCs the maximum and minimum signal values.

1. **REMOVE ALL PROPELLERS.**
2. Go to **Vehicle Setup > Sensors > ESC Calibration**. (If hidden, check the **Power** tab).
3. Follow the on-screen prompts: Unplug battery $\rightarrow$ Click Start $\rightarrow$ Plug in battery $\rightarrow$ Wait for initialization tones $\rightarrow$ Move throttle stick to zero.

### 46.2 Increasing Minimum Idle Speed

If a motor struggles to overcome internal friction upon arming, the baseline voltage is too low.

1. In Parameters, search for **`MOT_SPIN_ARM`**.
2. Increase the value incrementally (e.g., from `0.10` to `0.15`).
3. This applies higher baseline power to all motors immediately upon arming.

---

## 47. Unhiding Advanced Parameters in QGC

If critical configuration parameters (like `MOT_PWM_TYPE`) do not appear in the search results, QGroundControl is filtering the view to protect novice users.

1. Navigate to the **Parameters** screen.
2. In the top-left corner, beneath the search bar, click the dropdown menu currently labeled **Standard**.
3. Select **Full Parameter List** (or **Advanced**). All hidden ArduPilot parameters are now accessible.

### 47.1 Firmware-Specific Parameter Naming

If `MOT_` parameters remain missing in the Full Parameter List, the firmware version utilizes alternate naming conventions:

* Search for **`SERVO_BLH_OTYPE`** (Alternative for DShot configuration).
* Search for **`Q_M_`** (If you accidentally flashed QuadPlane firmware instead of ArduCopter, all motor parameters use this prefix).

---

## 48. Diagnosing Asymmetrical Motor Stuttering (Twitching)

If a motor twitches rapidly back and forth but fails to complete a revolution, it is **not** a software or calibration error. This is a hardware failure known as "Phase Loss." Brushless motors require three active electrical phases to generate a rotating magnetic field; if one phase drops, the motor stutters.

---

### 48.1 The Solder Joint Inspection

A "cold" solder joint creates high resistance or intermittent contact.

1. Inspect the three heavy-gauge wires connecting the stuttering motor to the ESC.
2. A good joint is shiny and smooth. A bad joint is dull, crystalline, or balled up.
3. **Fix:** Re-flow all three solder joints on that specific motor pad using flux and a high-heat soldering iron.

### 48.2 The 8-Pin Wiring Harness (FC to ESC)

On stack-based flight controllers like the SpeedyBee F405 V4, motor signals travel from the top board (FC) to the bottom board (ESC) via a delicate 8-pin JST/ribbon cable.

1. Unplug the cable from both the FC and the ESC.
2. Inspect the interior of the white plastic connectors.
3. **Fix:** If a single gold pin is bent, pushed backward, or corroded, the signal for that specific motor is dead. Straighten the pin or replace the entire cable harness.

### 48.3 The Mounting Screw Short

If the M3 screws used to mount the motor to the carbon fiber frame are too long, they will pierce the enamel coating of the copper stator windings inside the motor.

1. This creates a hard electrical short to the carbon frame, instantly causing phase loss and stuttering.
2. **Diagnostic Test:** Remove all four mounting screws. If the motor spins perfectly when unmounted, the screws are too long. Use shorter hardware or add washers.

### 48.4 The Motor Swap Test (Isolating the Dead Component)

To definitively prove whether the motor is burned out or the ESC MOSFET is destroyed:

1. Desolder the three wires of the stuttering motor.
2. Desolder a known-working motor from a different corner of the drone.
3. Solder the suspect motor to the working ESC pads.
* **If the suspect motor still stutters:** The motor's internal windings are burned or broken. Replace the motor.
* **If the suspect motor now spins perfectly (and the working motor stutters on the old pads):** The ESC board has a blown MOSFET on that specific output. The ESC board must be replaced.



---

Documentation complete. No further chat sections provided.
