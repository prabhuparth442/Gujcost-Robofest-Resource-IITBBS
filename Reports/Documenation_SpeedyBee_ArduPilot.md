# SpeedyBee ArduPilot: Complete Setup & Troubleshooting Manual
*Comprehensive Reference — All Sections, All Details, No Redundancy*
*Original Data : 
![Source Data](https://docs.google.com/document/d/1z3HKV97rB1eR1Wk653h4rTs9prCnCsXNmgBnI1QO64Q/edit?usp=sharing)

---

## Table of Contents

1. [Overview and Problem Statement](#section-1)
2. [SD Card Formatting (Linux)](#section-2)
3. [Hardware Initialization & Power Rules](#section-3)
4. [Secondary Fallback Parameters](#section-4)
5. [Wireless MAVLink Bridge via Raspberry Pi](#section-5)
6. [Raspberry Pi SSH & Network Troubleshooting](#section-6)
7. [Dual Network Setup via Netplan (Ethernet + Wi-Fi)](#section-7)
8. [Wi-Fi Troubleshooting via nmcli](#section-8)
9. [Launching MAVProxy & Wireless Calibration](#section-9)
10. [Resolving MAVProxy Python Dependencies](#section-10)
11. [Unlocking Hardware UART (`/dev/serial0` Error)](#section-11)
12. [Overcurrent Protection (3-Second Shutdown)](#section-12)
13. [Bypassing `/dev/serial0` (Direct UART Alias)](#section-13)
14. [Troubleshooting "Waiting for Heartbeat" (Link 1 Down)](#section-14)
15. [Resolving Persistent `Input/Output Error` (ttyS0)](#section-15)
16. [Resolving Hardware Silence (`b''` Output)](#section-16)
17. [Understanding Persistent Input/Output Error (Error 5)](#section-17)
18. [Finalizing Boot Configuration Files](#section-18)
19. [Mandatory Power Reset & Final Verification](#section-19)
20. [Diagnosing the I2C Bus Conflict](#section-20)
21. [Resolving I2C Hardware & Wiring Failures](#section-21)
22. [Firmware and Environmental Barometer Fixes](#section-22)
23. [Diagnosing ArduPilot Boot-Loops (I2C Hangs)](#section-23)
24. [Resolving Duplicate I2C Address Clashes (DPS310 Conflict)](#section-24)
25. [Resolving "Compass Not Healthy" Errors](#section-25)
26. [Unhiding Advanced Parameters in QGC](#section-26)
27. [Managing "Ghost" Compasses (I2C Clutter)](#section-27)
28. [Standard PreArm Failsafes](#section-28)
29. [Resolving "Cannot Start Compass Thread"](#section-29)
30. [UI-Induced Thread Crashes](#section-30)
31. [MAVLink Console Calibration Override](#section-31)
32. [Handling "Calibration Complete" but Persistent Red Errors](#section-32)
33. [In-Flight Calibration Fallback (`COMPASS_LEARN`)](#section-33)
34. [Radio Integration (FlySky FS-i6 & iA6B)](#section-34)
35. [Throttle Failsafes & `FS_THR_VALUE`](#section-35)
36. [Final Radio Calibration Sequence](#section-36)
37. [Final Arming Checklist](#section-37)
38. [Flight Modes & Arm Switch Configuration (FlySky FS-i6)](#section-38)
39. [Configuring ArduPilot for i-BUS](#section-39)
40. [Overriding Persistent Radio & Throttle Failsafes](#section-40)
41. [Configuring a Physical Arming Switch](#section-41)
42. [Resolving Motor Rotation Failures](#section-42)
43. [Resolving "Logging Failed" Pre-Arm Errors](#section-43)
44. [Resolving Symmetrical Throttle Lag (Slow Motor Spin)](#section-44)
45. [Unhiding Advanced Parameters in QGC (Motor Context)](#section-45)
46. [Diagnosing Asymmetrical Motor Stuttering (Phase Loss)](#section-46)
47. [Appendix — Known Inconsistencies & Corrections Log](#appendix)

---

## Background: What Is ArduPilot and Why Is This Complex?

ArduPilot is not simple drone firmware — it is a full real-time operating system (RTOS) designed for autonomous vehicles. Unlike basic flight controllers that run a minimal control loop, ArduPilot:

- Requires an SD card to allocate memory, store parameters, and run initialization scripts on every boot.
- Manages multiple hardware buses simultaneously (SPI, I2C, UART) and will abort the entire boot sequence if any bus fails to respond in time.
- Enforces strict pre-arm safety checks across sensors, radio, power, and logging — none of which can be skipped without explicit parameter overrides.
- Has a layered parameter system where many critical settings are hidden from the default QGroundControl interface.

**QGroundControl (QGC)** is the ground station software used to configure and monitor ArduPilot. It connects via USB or wirelessly over MAVLink (a lightweight telemetry protocol).

**SpeedyBee F405 V3/V4** is an All-in-One (AIO) flight controller — meaning the flight controller and Electronic Speed Controller (ESC) are stacked together as one unit. This design has unique power and wiring implications covered throughout this manual.

---

<a name="section-1"></a>
## Section 1 — Overview and Problem Statement

### The Primary Symptom

When running ArduPilot on a SpeedyBee F405 V3 or V4 via QGroundControl, a common failure is an infinite boot loop displaying:

> **Config Error: Fix problem then reboot**

The QGC Messages tab typically cites a Barometer failure:
```
Baro: unable to initialise driver
```

### Why This Happens

ArduPilot operates as a real-time OS and requires a properly formatted SD card to allocate memory, load parameters, and run initialization scripts. A missing, corrupted, or wrongly partitioned SD card causes a boot timeout. This timeout prevents sensors like the Barometer from initializing in time, which triggers the Config Error boot loop.

### All Known Root Causes

This manual addresses every root cause identified through real hardware builds:

| Root Cause | Section |
|-----------|---------|
| Missing, corrupted, or wrongly partitioned SD card | Section 2 |
| USB-only power causing brownouts | Section 3 |
| I2C Bus Jamming from a miswired GPS module | Section 20 |
| Duplicate I2C addresses (GPS module contains its own DPS310 barometer) | Section 24 |
| Raspberry Pi UART not unlocked or reserved by OS | Section 11 |
| Background Linux services stealing the serial port | Sections 11, 14, 15 |
| Boot configuration typos or missing entries | Section 18 |

---

<a name="section-2"></a>
## Section 2 — SD Card Formatting (Linux)

### Why a Standard Format Is Not Enough

ArduPilot requires a single FAT32 partition with a **32KB cluster size** and a clean DOS partition table. Standard operating system formats (Windows default, macOS Disk Utility) often:

- Create multiple partitions (including hidden EFI/recovery partitions).
- Use cluster sizes that are too small, causing read timeouts.
- Format as exFAT, which the SpeedyBee F405 V4 **cannot read at all**.

> **Important hardware limit:** The SpeedyBee F405 V4 cannot read exFAT. The card must be **32GB or smaller** and formatted strictly as FAT32.

---

### Step 2.1 — Identify the SD Card

Plug the SD card into your Linux machine. Open a terminal and list block devices:

```bash
lsblk
```

Look for your SD card (commonly `/dev/sda`, `/dev/sdb`, or `/dev/mmcblk0`). In a typical build scenario the card appears as `/dev/sda` with two conflicting partitions (`sda1`, `sda2`).

> **Warning:** Double-check the device name. All data on the target device will be destroyed. Never run these commands on `/dev/sda` if that is your system's main hard drive — verify first.

---

### Step 2.2 — Unmount All Partitions

You cannot format a drive while it is mounted. Unmount every partition:

```bash
sudo umount /dev/sda1
sudo umount /dev/sda2
```

If additional partitions exist (e.g., `sda3`), unmount those too. If a partition is not mounted, the command will return an error — this is harmless, continue.

---

### Step 2.3 — Wipe and Rebuild the Partition Table with `fdisk`

ArduPilot will fail if multiple partitions exist. You must delete all of them and create exactly one Windows 95 FAT32 (LBA) partition.

```bash
sudo fdisk /dev/sda
```

Inside the `fdisk` interactive prompt, type each of the following keys and press **Enter** after each:

| Key | What It Does |
|-----|-------------|
| `o` | Creates a new, empty DOS partition table, wiping all existing partitions |
| `n` | Creates a new partition |
| `p` | Selects "Primary" partition type |
| `1` | Sets partition number to 1 |
| Enter | Accepts default first sector (start of disk) |
| Enter | Accepts default last sector (end of disk — uses entire card) |
| `t` | Opens the partition type selector |
| `c` | Sets the type to W95 FAT32 (LBA) — hex code `0c` |
| `w` | Writes all changes to the disk and exits `fdisk` |

After `w`, `fdisk` exits and the partition table is written.

---

### Step 2.4 — Format with a 32KB Cluster Size

The partition now exists but has no filesystem. Run:

```bash
sudo mkfs.vfat -F 32 -s 64 -n "SPEEDYBEE" /dev/sda1
```

**Parameter breakdown:**

| Flag | Meaning |
|------|---------|
| `-F 32` | Forces FAT32 filesystem (not FAT16 or FAT12) |
| `-s 64` | Sets sectors-per-cluster to 64. At 512 bytes per sector: 64 × 512 = **32,768 bytes = 32KB clusters** |
| `-n "SPEEDYBEE"` | Sets the volume label (optional but useful for identification) |
| `/dev/sda1` | Targets the newly created partition — not the whole drive |

---

<a name="section-3"></a>
## Section 3 — Hardware Initialization & Power Rules

### The Single Power Rule

> **This is the most commonly violated rule and causes the most hardware damage.**

Never supply 5V to the Raspberry Pi via USB-C if it is also receiving 5V from the drone's UBEC (powered by the LiPo battery). This creates a voltage conflict and a ground loop, triggering the SpeedyBee's Overcurrent Protection — the board will shut down after approximately 3 seconds.

**Correct power assignments:**

| Component | Power Source | Notes |
|-----------|-------------|-------|
| Raspberry Pi | Drone UBEC (from LiPo battery) | USB-C cable must be physically disconnected |
| SpeedyBee F405 | LiPo battery via XT60 or XT30 | Do not use USB-only for flight operations |
| Laptop/Ground Station | Own power | Connect to Pi over Wi-Fi SSH (Section 7) |

**Why USB-only power is insufficient for the SpeedyBee:**
USB ports typically supply a maximum of 500mA at 5V (2.5W). The SpeedyBee's Barometer, compass, SD card reader, and GPS draw current spikes that can exceed this. Voltage drops (brownouts) during these spikes prevent the Barometer from initializing, directly causing the `Baro: unable to initialise driver` error.

---

### Step 3.1 — Verify the Fix

Insert the formatted SD card, connect the LiPo battery, then connect USB. If the `Config Error` disappears, the fix worked.

To manually verify the Barometer is healthy:

1. Go to the **Analyze** tab in QGC → **MAVLink Console**.
2. Type `status` and press Enter.
3. Look for `Baro: 1 OK` in the output. This confirms the sensor is actively communicating.

---

<a name="section-4"></a>
## Section 4 — Secondary Fallback Parameters

If formatting the SD card and using LiPo power do not clear the error, check these parameters in **QGC → Vehicle Setup → Parameters**:

| Parameter | Correct Value | Reason |
|-----------|--------------|--------|
| `BARO_PROBE_EXT` | `0` | Stops ArduPilot from searching for a non-existent external GPS barometer. If it hunts for one and times out, it crashes. |
| `BARO_PRIMARY` | `0` | Ensures ArduPilot uses the onboard barometer as primary |
| `BRD_IO_ENABLE` | `0` | SpeedyBee AIO boards do not have a secondary I/O co-processor. Leaving this enabled causes ArduPilot to wait for hardware that doesn't exist. |

**Additional fallback steps:**

- **Reset to firmware defaults:** In QGC Parameters screen → Tools → **Reset all to firmware's defaults**. This clears "zombie parameters" left over from previous firmware versions that can cause unpredictable conflicts.

- **Hardware inspection:** Examine the Barometer chip under magnification. It is the small silver rectangular component on the SpeedyBee board with a tiny hole in the top (the pressure inlet). Inspect for stray solder balls or microscopic wire strands bridging adjacent pins.

---

<a name="section-5"></a>
## Section 5 — Wireless MAVLink Bridge via Raspberry Pi

### What This Section Achieves

A Raspberry Pi reads serial telemetry data from the SpeedyBee via UART and rebroadcasts it over Wi-Fi as UDP packets to a laptop running QGroundControl. This eliminates the USB tether during calibration and field testing, allowing free physical movement of the drone for accelerometer and compass calibration.

---

### Step 5.1 — Hardware Wiring (UART6)

Connect the Raspberry Pi's GPIO UART pins to the SpeedyBee's UART6 pads (`T6` and `R6`).

> **⚠ CRITICAL:** Do NOT connect the SpeedyBee's 5V pin to the Raspberry Pi if the Pi has its own USB-C power. This causes a ground loop and destroys hardware (see Section 3).

| Raspberry Pi Pin | GPIO Name | SpeedyBee Pad | Notes |
|-----------------|-----------|--------------|-------|
| Pin 6 | GND | GND | Mandatory shared ground — do not skip |
| Pin 8 | GPIO 14 (TX) | **R6** (UART6 RX) | Pi transmits → SpeedyBee receives |
| Pin 10 | GPIO 15 (RX) | **T6** (UART6 TX) | SpeedyBee transmits → Pi receives |

> **Why UART6 specifically?** UART6 (pads labeled T6/R6) is the dedicated port used in this build. Earlier documentation incorrectly referenced UART4. This manual standardizes on UART6 throughout.

> **TX→RX crossing rule:** Serial (UART) communication always crosses — the Transmit pin of one device connects to the Receive pin of the other. This is the opposite of I2C (see Section 21).

---

### Step 5.2 — ArduPilot Parameters (UART6)

These parameters must be set while the SpeedyBee is still connected via USB (before switching to wireless). Connect to QGC via USB one final time.

In **QGC → Vehicle Setup → Parameters**, search for and set:

```
SERIAL6_PROTOCOL = 2    (MAVLink 2 — the standard protocol)
SERIAL6_BAUD     = 921  (represents 921600 baud)
```

> **Note on baud notation:** QGC displays baud rates divided by 100. The value `921` means 921,600 baud. This is required for reliable high-frequency telemetry over UART.

**If a GPS was previously configured on UART6:**
- Set `GPS_TYPE = 0` for UART6 to disable GPS on that port.
- Physically move the GPS connector to UART1, UART2, or UART3.
- Write parameters and reboot before disconnecting USB.

---

### Step 5.3 — Raspberry Pi Software Setup

Modern Linux distributions (Debian Bookworm and later) enforce **PEP 668**, which prevents installing Python packages globally with `pip` to protect system-managed packages. You must use a Python virtual environment.

**Step-by-step:**

```bash
# Update package lists and install virtual environment tools
sudo apt update && sudo apt install python3-venv python3-full

# Create a project directory and virtual environment
mkdir ~/drone_link && cd ~/drone_link
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Install MAVProxy and all required dependencies
pip install mavproxy future pyserial pymavlink
```

> **Why install `future`, `pyserial`, and `pymavlink` separately?** On Python 3.12+, MAVProxy's automatic dependency resolution sometimes fails to pull these in. Installing them explicitly together prevents cascading `ModuleNotFoundError` crashes at runtime. Common errors without these:
> - `ModuleNotFoundError: No module named 'future'`
> - `ModuleNotFoundError: No module named 'serial'`
> - `ModuleNotFoundError: No module named 'pymavlink'`

---

### Step 5.4 — Serial Port Permissions

By default, the Raspberry Pi's standard user cannot access serial ports (`/dev/ttyAMA0`, etc.). You must add the user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

**You must reboot the Pi for this group membership change to take effect.** The command succeeds silently — if you skip the reboot and try to open the serial port, you will get a `Permission denied` error.

---

### Step 5.5 — Running the Wireless Bridge

Find your laptop's current IP address:
- **Linux:** `hostname -I | awk '{print $1}'`
- **Windows:** Open Command Prompt → `ipconfig` → look for "IPv4 Address"

Example output: `10.119.242.128`

On the Raspberry Pi, start the bridge:

```bash
source ~/drone_link/venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:10.119.242.128:14550
```

Replace `10.119.242.128` with your actual laptop IP. MAVProxy will print `Waiting for heartbeat` until the SpeedyBee is powered on with LiPo.

---

### Step 5.6 — Connecting QGroundControl

1. Open QGC on the laptop.
2. Click the **Q icon (top-left) → Application Settings → Comm Links → Add**.
3. Set **Name** to any descriptive label (e.g., `Drone_WiFi`).
4. Set **Type** to `UDP`.
5. Set **Port** to `14550`.
6. Click **OK**, select the new link, and click **Connect**.
7. Verify connection by typing `status` in the MAVLink Console (Analyze tab) and confirming `Baro: 1 OK`.

---

<a name="section-6"></a>
## Section 6 — Raspberry Pi SSH & Network Troubleshooting

### Step 6.1 — SSH Connection Refused After Reboot

If `ssh` returns `Connection refused` immediately after the Pi reboots:

1. **Wait 60 seconds.** The SSH daemon (`sshd`) is typically the last service to load during boot. The Pi may be up on the network but not yet accepting SSH connections.

2. **Connect via mDNS hostname** instead of IP (which may have changed):
   ```bash
   ssh drone2@drone2.local
   ```
   Replace `drone2` with your Pi's configured hostname.

3. **Check the onboard LEDs:**
   - Solid red = Power is good, 5V rail is stable.
   - Flashing green = OS is actively reading the SD card (booting).
   - No green flash at all = OS is not booting. Check the Pi's SD card.

---

### Step 6.2 — Enabling SSH on a Headless Pi (No Monitor)

If SSH was never enabled or was disabled, you can force it to activate without connecting a monitor or keyboard:

1. Remove the SD card from the Pi and insert it into a laptop or desktop.
2. Open the `boot` partition (also called `bootfs` on newer Pi OS versions).
3. Create a blank file named exactly **`ssh`** with no file extension:
   ```bash
   touch /path/to/boot/ssh
   ```
   On Windows, use Notepad to create a new file and use "Save As" with `ssh` as the filename, selecting "All Files" to prevent `.txt` being appended.
4. Reinsert the SD card into the Pi and power it on.
5. Once you connect via SSH, permanently enable the service so it survives future reboots:
   ```bash
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

---

<a name="section-7"></a>
## Section 7 — Dual Network Setup via Netplan (Ethernet + Wi-Fi)

### Why This Setup Is Needed

During initial configuration, you are connected to the Pi over Ethernet (wired SSH). Adding Wi-Fi incorrectly can drop the Ethernet session and lock you out. Netplan's `try` command provides a safety net.

### Step 7.1 — Edit the Netplan Configuration File

```bash
# Find the name of your current Netplan config file
ls /etc/netplan/

# Open it for editing (replace YOUR_FILE_NAME with the actual filename, e.g., 50-cloud-init.yaml)
sudo nano /etc/netplan/YOUR_FILE_NAME.yaml
```

Modify the file to **exactly** this structure. **Use the Spacebar for indentation — never the Tab key.** YAML is indentation-sensitive and tabs will cause a parse failure.

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

The `optional: true` setting is critical — it tells the Pi to continue booting even if either network interface is not available. Without it, the Pi can hang for 60–120 seconds at boot trying to acquire a lease.

Save with `Ctrl+O`, `Enter`. Exit with `Ctrl+X`.

---

### Step 7.2 — Apply with the Safety-Net Command

```bash
sudo netplan try
```

**Do not use `sudo netplan apply`.** The `try` command applies the new configuration but automatically reverts to the previous working configuration after **120 seconds** if you do not confirm. This prevents being permanently locked out over Ethernet if the new config has an error.

When prompted, type `yes` and press Enter if Ethernet is still working.

---

<a name="section-8"></a>
## Section 8 — Wi-Fi Troubleshooting via nmcli

If connecting to Wi-Fi via Netplan fails (e.g., "No network found" or the SSID isn't visible), the Pi's Wi-Fi radio may be soft-blocked by the OS or out of sync.

### Step 8.1 — Force Unblock and Rescan

Run these commands in sequence:

```bash
sudo nmcli radio wifi on       # Enables the Wi-Fi radio at the software level
sudo rfkill unblock wifi       # Removes any kernel-level soft block on the radio
sudo nmcli device wifi rescan  # Forces the radio to scan for available networks
```

Wait approximately 5 seconds for the scan to complete, then list visible networks:

```bash
nmcli device wifi list
```

Confirm your SSID is visible before attempting to connect.

---

### Step 8.2 — Force Connection to a Specific Network

If the network is visible but not auto-connecting, or if it is a hidden SSID:

```bash
sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD" hidden yes
```

The `hidden yes` flag makes the Pi actively probe for the network even if it is not broadcasting its SSID. This can also be used for normal visible networks as a more aggressive connection method.

---

<a name="section-9"></a>
## Section 9 — Launching MAVProxy & Wireless Calibration

### Step 9.1 — Start the Bridge on the Raspberry Pi

```bash
source ~/drone_link/venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:10.119.242.128:14550
```

Replace the IP address with your laptop's current IP if it has changed since initial setup. Check for the `Heartbeat` message in the MAVProxy terminal — this confirms the SpeedyBee is transmitting MAVLink data.

---

### Step 9.2 — Connect QGroundControl

Open QGC on the laptop. Go to **Application Settings → Comm Links** and activate the UDP link on Port `14550` created in Section 5.6.

---

### Step 9.3 — Perform Wireless Accelerometer Calibration

With the drone untethered from USB, you can freely rotate it for calibration.

1. In QGC, go to **Vehicle Setup (gear icon) → Sensors → Accelerometer**.
2. Click **Calibrate**.
3. Follow the on-screen prompts — place the drone in each of 6 positions and hold still when indicated:
   - Level (top facing up, nose facing forward)
   - Left Side (rotate 90° so left side faces down)
   - Right Side (rotate 90° so right side faces down)
   - Nose Down (pitch forward 90° so nose faces down)
   - Nose Up (pitch backward 90° so nose faces up)
   - Upside Down (flip 180° so bottom faces up)
4. QGC will confirm each position with a green checkmark before prompting for the next.

If a compass is attached, repeat the process under the **Compass** tab, rotating the drone smoothly through all axes until the progress bar completes (see Section 25 for detailed compass calibration).

---

<a name="section-10"></a>
## Section 10 — Resolving MAVProxy Python Dependencies

When launching MAVProxy inside a virtual environment on Python 3.12 or newer, you may encounter dependency crashes immediately on startup.

### Common Error Messages

```
ModuleNotFoundError: No module named 'future'
ModuleNotFoundError: No module named 'serial'
ModuleNotFoundError: No module named 'pymavlink'
```

### Fix — Install All Required Modules Together

Ensure the virtual environment is active, then install:

```bash
source ~/drone_link/venv/bin/activate
pip install mavproxy future pyserial pymavlink
```

Installing all four in a single command ensures pip resolves version compatibility between them simultaneously. Installing them one at a time can result in version conflicts.

---

<a name="section-11"></a>
## Section 11 — Unlocking Hardware UART (`/dev/serial0` Error)

### Error Message

```
[Errno 2] could not open port /dev/serial0: [Errno 2] No such file or directory: '/dev/serial0'
```

### What This Means

The physical RX/TX GPIO pins (Pins 8 and 10) on the Raspberry Pi are either:
1. Disabled at the hardware/boot level, or
2. Reserved by the OS for a serial login console (so you can plug in a terminal to the GPIO pins to log in).

On standard Raspberry Pi OS, `raspi-config` handles this setup. On Ubuntu distributions, you must manually edit the boot configuration files.

### The Two UARTs on Raspberry Pi — Important Background

The Raspberry Pi has two UART hardware interfaces:
- **Mini-UART (ttyS0):** A simpler, lower-quality UART whose baud rate is tied to the CPU clock frequency. It **cannot reliably sustain 921600 baud** — data corruption is common.
- **PL011 UART (ttyAMA0):** The full-featured, hardware-clocked UART capable of stable high-speed communication. This is what MAVProxy needs.

By default, the Pi assigns the PL011 UART to the Bluetooth chip, and the Mini-UART to the GPIO pins. You must swap this assignment by disabling Bluetooth.

---

### Step 11.1 — Enable UART and Disable Bluetooth (`config.txt`)

```bash
sudo nano /boot/firmware/config.txt
# If that file is empty or doesn't exist, try:
# sudo nano /boot/config.txt
```

Scroll to the **absolute bottom** of the file. Append these lines exactly to the `[all]` section:

```
[all]
enable_uart=1
dtoverlay=disable-bt
```

> **⚠ Critical typo warning:** The overlay name must be `disable-bt` — not `disable-b`, not `disable_bt`, not `disablebt`. Any deviation will silently fail, leaving Bluetooth assigned to the PL011 UART and your GPIO pins still running the slower Mini-UART.

`enable_uart=1` explicitly activates the GPIO UART pins. `dtoverlay=disable-bt` loads a device tree overlay that disconnects the Bluetooth chip from the PL011 UART and reassigns PL011 to the GPIO pins.

---

### Step 11.2 — Strip the Serial Console (`cmdline.txt`)

```bash
sudo nano /boot/firmware/cmdline.txt
# Or: sudo nano /boot/cmdline.txt
```

> **Critical:** This file contains exactly **one continuous line of text**. Do not add any line breaks. Adding a newline will prevent the Pi from booting.

Locate and delete **only** the following string (one of these forms):
- `console=serial0,115200`
- `console=ttyAMA0,115200`
- `console=ttyS0,115200`

**Do NOT delete `console=tty1`** — this is the HDMI/monitor console and must remain.

A valid, clean `cmdline.txt` looks like:
```
multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IN
```

Save with `Ctrl+O`, `Enter`. Exit with `Ctrl+X`.

---

### Step 11.3 — Remove and Mask Squatter Services

Two types of background services will steal serial port access from MAVProxy:

**ModemManager** scans open serial ports looking for 4G/LTE modems. When it scans the port connected to your SpeedyBee, it sends modem probe commands that corrupt the MAVLink data stream and cause a crash. Remove it completely:

```bash
sudo apt-get remove modemmanager -y
```

**Serial Getty services** provide login terminals on serial ports — exactly what we removed from `cmdline.txt`. The OS may still attempt to respawn them via systemd unless explicitly masked:

```bash
sudo systemctl mask serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyS0.service
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyS0.service
```

`mask` is stronger than `disable` — it creates a symlink to `/dev/null` that prevents the service from ever starting, even if another service tries to start it.

---

### Step 11.4 — Reboot and Verify

```bash
sudo reboot
```

Wait 60 seconds, reconnect via SSH, then verify:

```bash
ls -l /dev/serial0
```

**Expected output:** A symlink pointing to `ttyAMA0`:
```
lrwxrwxrwx 1 root root 7 Jan  1 00:00 /dev/serial0 -> ttyAMA0
```

Also confirm available serial devices:
```bash
ls /dev/ttyA* /dev/ttyS*
```

---

<a name="section-12"></a>
## Section 12 — Overcurrent Protection (3-Second Shutdown)

### Symptom

The SpeedyBee powers on (LEDs illuminate), then shuts down completely after 2–3 seconds.

### Cause

The Power Management IC (PMIC) on the SpeedyBee detects either a short circuit or a voltage conflict and triggers its protection circuit. This is a hardware safety feature that cuts power to prevent component damage.

### Action Steps

1. **Unplug all power immediately.** Do not repeatedly test by plugging the battery back in — each overcurrent event stresses the voltage regulators. Repeated testing destroys them.

2. **Inspect solder joints** on the `T6`, `R6`, `5V`, and `GND` pads under magnification (phone camera zoom works). Look for:
   - Microscopic solder balls bridging adjacent pads
   - Stray wire strands shorting across pads
   - Solder bridges between any two adjacent pads

3. **Enforce the Single Power Rule** (Section 3):
   - Raspberry Pi must receive 5V from exactly one source.
   - If the Pi is connected to the drone's UBEC via the 5V pad, the USB-C cable must be physically disconnected.
   - If the Pi is powered by USB-C, the 5V wire to the UBEC must be disconnected.

---

<a name="section-13"></a>
## Section 13 — Bypassing `/dev/serial0` (Direct UART Alias)

If the `serial0` symlink still doesn't exist after rebooting with the boot file edits applied, point MAVProxy directly to the hardware device instead:

### Step 13.1 — Confirm the Hardware Port Exists

```bash
ls /dev/ttyA* /dev/ttyS*
```

If `/dev/ttyAMA0` is listed, the PL011 UART hardware is present and accessible.

### Step 13.2 — Launch MAVProxy Using Direct Alias

```bash
source ~/drone_link/venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:10.119.242.128:14550
```

Using `ttyAMA0` directly bypasses the symlink and communicates directly with the PL011 UART hardware.

---

<a name="section-14"></a>
## Section 14 — Troubleshooting "Waiting for Heartbeat" (Link 1 Down)

### What This Means

MAVProxy successfully opened the serial port — the software path is working. However, the SpeedyBee is not transmitting any MAVLink data. The Pi is listening but the flight controller is silent.

Work through these steps in order:

---

### Step 14.1 — Wiring Cross-Check

Serial communication requires TX → RX crossing. Verify:
- Pi **Pin 8 (TX / GPIO 14)** → SpeedyBee **R6**
- Pi **Pin 10 (RX / GPIO 15)** → SpeedyBee **T6**

**The Mandatory Swap Test:** If wired exactly as above but still failing, physically swap the two data wires at the SpeedyBee pads (connect what was on R6 to T6, and vice versa). Board pad labeling is **frequently inverted by manufacturers** — what the board labels "T6" may actually be functioning as the receive pin internally. This swap fixes the issue in a significant percentage of cases.

---

### Step 14.2 — Kill Squatter Services

If services were not masked in Section 11.3, do it now:

```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service
sudo apt-get remove modemmanager -y
```

---

### Step 14.3 — Verify ArduPilot Parameters

Connect the SpeedyBee to QGC via USB and confirm these parameters are set correctly:

| Parameter | Required Value | Notes |
|-----------|---------------|-------|
| `SERIAL6_PROTOCOL` | `2` | MAVLink 2 protocol |
| `SERIAL6_BAUD` | `921` | 921600 baud |
| `BRD_SER6_RTSCTS` | `0` | Disables hardware flow control (RTS/CTS). Enabling this when no RTS/CTS wires are connected causes the FC to wait forever for a clear-to-send signal. |

Write parameters and reboot the flight controller after any changes.

---

### Step 14.4 — The Raw Python Diagnostic Test

This test bypasses MAVProxy entirely to isolate whether the problem is in the software layer or the physical hardware:

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0', 921600, timeout=1); print(s.read(100))"
```

**Interpreting the output:**

| Output | Meaning | Next Step |
|--------|---------|-----------|
| Hex/binary data (e.g., `b'\xfe\x01\x05...'`) | Hardware link is perfect. SpeedyBee is transmitting. | Problem is in MAVProxy config — recheck parameters and port settings. |
| Empty string `b''` | Hardware link is dead. SpeedyBee is not transmitting. | Check physical wiring, ArduPilot parameters, and LiPo power. |
| `Permission denied` | User lacks serial port access | Run `sudo usermod -a -G dialout $USER` and reboot |
| `[Errno 2] No such file...` | Port doesn't exist | Return to Section 11 |
| `Input/output error` | OS has locked the port | Go to Section 15 |

---

<a name="section-15"></a>
## Section 15 — Resolving Persistent `Input/Output Error` (ttyS0)

### Error Message

```
[Errno 5] Input/output error
```

### Cause

The Linux kernel has monopolized the serial hardware for a login console. When MAVProxy or a Python script attempts to open the port, the OS denies access — it treats the serial port as a system resource, not a user-accessible device. Stopping `serial-getty` via `systemctl stop` alone is insufficient on some Ubuntu builds because the kernel boot configuration actively re-engages it.

### Fix

You must both mask the systemd services **and** edit the boot file:

```bash
sudo systemctl mask serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl stop serial-getty@ttyAMA0.service
```

Then edit `cmdline.txt` to ensure the serial console assignment is completely removed (see Section 11.2 for exact instructions). After editing, reboot the Pi — the kernel must release the hardware lock during boot.

---

<a name="section-16"></a>
## Section 16 — Resolving Hardware Silence (`b''` Output)

When the serial port opens without error but the Python test returns `b''` (empty), the software is working correctly — the problem is entirely physical.

### Step 16.1 — The Mandatory Wire Swap

Disconnect the data wires from the SpeedyBee pads and reconnect them in reverse:
- Wire that was on `R6` → move to `T6`
- Wire that was on `T6` → move to `R6`

Manufacturer pad labeling inconsistency is the single most common cause of `b''` output.

---

### Step 16.2 — Verify Common Ground

Serial communication measures voltage differentials between data lines and ground. If the Pi and SpeedyBee do not share a ground reference, the data signal appears as noise or nothing.

Ensure a dedicated wire connects a **GND pin on the Raspberry Pi** directly to a **GND pad on the SpeedyBee**. Do not rely on the battery negative terminal as an indirect ground path — resistance and inductance in power cables at high baud rates degrades signal integrity.

---

### Step 16.3 — Check Hidden ArduPilot Signal Options

ArduPilot has a parameter that enables signal inversion and half-duplex modes. If these were accidentally enabled (they affect signal polarity), the data appears as inverted noise:

Connect the SpeedyBee via USB to QGC and verify:

```
SERIAL6_OPTIONS = 0   (All options disabled — no inversion, no half-duplex)
```

Write parameters and power cycle the flight controller using the LiPo battery (not just a software reboot).

---

### Step 16.4 — Test Alternate UART Aliases

On specific Ubuntu builds, the high-speed UART may be assigned a non-standard device node. If `ttyAMA0` and `ttyS0` both fail, test:

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA1', 921600, timeout=1); print(s.read(100))"
```

---

<a name="section-17"></a>
## Section 17 — Understanding Persistent Input/Output Error (Error 5)

### Two Distinct Failure Modes

**Error 5 on `/dev/ttyS0` (Input/Output Error):**
The OS is actively listening for a user to log in via the TX/RX GPIO pins. When MAVProxy or Python attempts to open the port, the kernel denies access. Fix: Edit `cmdline.txt` and mask the serial-getty services, then reboot.

**`b''` (empty output) on `/dev/ttyAMA0`:**
The high-speed PL011 UART is currently assigned to the onboard Bluetooth chip rather than the GPIO pins. The port opens (no error) but the Bluetooth chip's data is not what you want. Fix: Apply the `config.txt` changes in Section 11.1 (`disable-bt` overlay) and reboot.

Both issues must be resolved simultaneously via the boot configuration changes in Section 18.

---

<a name="section-18"></a>
## Section 18 — Finalizing Boot Configuration Files

These two files are the definitive hardware allocation instructions that the Pi processes the instant it receives power:

### Step 18.1 — Strip the Serial Console (`cmdline.txt`)

```bash
sudo nano /boot/firmware/cmdline.txt
```

Verify the entire file is **one single continuous line** with no line breaks. Delete any instance of:
- `console=serial0,115200`
- `console=ttyAMA0,115200`
- `console=ttyS0,115200`

Keep `console=tty1` intact. A valid configuration:

```
multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IN
```

---

### Step 18.2 — Re-route the High-Speed UART (`config.txt`)

```bash
sudo nano /boot/firmware/config.txt
```

Scroll to the `[all]` section at the very bottom. Add:

```
[all]
enable_uart=1
dtoverlay=disable-bt
```

> **Mandatory typo check:** It must be `disable-bt` with a hyphen before `bt`. Any other spelling fails silently. The `t` in `bt` stands for "Bluetooth" and must be present.

---

<a name="section-19"></a>
## Section 19 — Mandatory Power Reset & Final Verification

Boot file changes are processed at the BIOS/hardware-init level and **cannot** be applied to a live running system. A full reboot is mandatory.

Additionally, the SpeedyBee must be power-cycled to clear any crashed I2C bus state that may have built up from previous failed boots.

### Step 19.1 — Execution Sequence

1. **Unplug the LiPo battery** from the drone (SpeedyBee powers down completely).
2. On the Raspberry Pi, issue a clean reboot:
   ```bash
   sudo reboot
   ```
3. Wait **60 seconds** for the Pi to fully boot and reconnect to Wi-Fi.
4. Re-establish SSH connection:
   ```bash
   ssh drone2@drone2.local
   ```
5. **Plug the LiPo battery back in.** The SpeedyBee will boot cleanly without interference from the Pi's initialization traffic.

### Step 19.2 — Final Data Verification

```bash
source ~/drone_link/venv/bin/activate
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0', 921600, timeout=1); print(s.read(100))"
```

| Result | Interpretation | Action |
|--------|---------------|--------|
| `b'\xfd\x01\x05...'` (hex bytes) | ✅ Hardware link fully verified | Proceed to MAVProxy bridge |
| `b''` (empty) | ❌ Physical hardware link dead | Swap TX and RX wires on SpeedyBee pads immediately |

---

<a name="section-20"></a>
## Section 20 — Diagnosing the I2C Bus Conflict

### New Root Cause: I2C Bus Jamming

Even with a perfect SD card and correct power, the `Baro: unable to initialise driver` error can persist if the external GPS/Compass is miswired. This is because the onboard barometer (DPS310 or SPL06) and the external compass inside the GPS puck both communicate over the same physical data lines — the I2C bus (SDA and SCL pads).

When a miswired or shorted GPS is connected, it can pull the I2C bus to ground, crashing it entirely. The barometer cannot respond, ArduPilot times out, and the `Config Error` loop restarts.

### The Definitive Diagnostic Test

1. **Unplug the GPS/Compass module completely** from the SpeedyBee.
2. Boot the flight controller.
3. **Interpret the result:**
   - **`Config Error` disappears entirely** → The GPS was the only problem. The SD card and parameters are fine.
   - **`Config Error` disappears but `Compass not healthy` appears** → Absolute confirmation of an I2C hardware conflict caused by the GPS connection.
   - **`Config Error` persists with GPS unplugged** → The GPS was not the cause. Return to Sections 2–4.

---

<a name="section-21"></a>
## Section 21 — Resolving I2C Hardware & Wiring Failures

### Step 21.1 — The Non-Crossing Rule for I2C

I2C wiring is fundamentally different from UART wiring:

| Protocol | Wiring Rule |
|----------|------------|
| UART (Serial) | **Crosses:** TX connects to RX on the other device |
| I2C | **Does NOT cross:** SDA connects to SDA; SCL connects to SCL |

Wiring the GPS SDA to SpeedyBee SCL (or vice versa) will crash the I2C bus immediately.

---

### Step 21.2 — BZGNSS BZ-251 Ribbon Cable Pinout

The BZ-251 GPS module uses a ribbon (flat flex) cable. Pin 1 is typically marked with a small triangle or white stripe on the connector. Ribbon cables are highly susceptible to crossover errors if inserted backwards.

| Pin | Signal | SpeedyBee F405 V4 Pad |
|-----|--------|-----------------------|
| 1 | TX (GPS transmit) | **R6** (UART6 RX — remember: TX crosses to RX) |
| 2 | RX (GPS receive) | **T6** (UART6 TX — RX crosses to TX) |
| 3 | GND | **G** (any ground pad) |
| 4 | 5V / VCC | **4V5** or **5V** pad (see voltage note below) |
| 5 | SCL | **SCL** |
| 6 | SDA | **SDA** |

> **Voltage Note — Critical:** On many F405 boards:
> - `4V5` pads receive power via USB (active without LiPo).
> - `5V` pads **only receive power when the LiPo battery is connected**.
>
> If your GPS is connected to a `5V` pad, the compass (and therefore the barometer via I2C bus contamination) will fail on USB-only power. If calibration works with LiPo but fails on USB, this is the reason.

---

### Step 21.3 — Inspect for Micro-Shorts

The `SCL` and `SDA` pads on the SpeedyBee F405 V4 are **directly adjacent** to each other. Use a magnifying glass or phone camera at maximum zoom. Look for:

- A single strand of wire from the ribbon cable that touches both pads
- A solder bridge (a tiny dome of solder connecting both pads)
- Flux residue that has become conductive

Even a high-resistance partial bridge can corrupt I2C communication without appearing as an obvious short.

---

<a name="section-22"></a>
## Section 22 — Firmware and Environmental Barometer Fixes

### If Wiring Is Correct but Conflict Persists

#### Step 22.1 — Force DPS310 Chip ID (SpeedyBee F405 V4)

The F405 V4 uses a DPS310 barometer. ArduPilot's automatic detection can occasionally fail to initialize this specific chip variant.

In QGC **Vehicle Setup → Parameters**:
- Set `BARO_PROBE_EXT` = `1` (enables external barometer probing with DPS310 bitmask)
- Set `BARO_OPTIONS` = `1`
- Reboot the flight controller

> **Note:** This differs from the standard fix (`BARO_PROBE_EXT = 0`). Only use these values if you have confirmed correct wiring and the barometer chip specifically fails to initialize.

---

#### Step 22.2 — Legacy Compass Driver Address Conflicts

Older GPS modules using HMC5883L compass chips may have I2C addresses that conflict with the onboard barometer.

In QGC Parameters, search for `COMPASS_TYPEMASK`. Disable legacy compass drivers one at a time to identify which driver is causing the bus conflict.

> **Version note:** `COMPASS_TYPEMASK` is deprecated in ArduPilot V4.5+. In V4.5 and newer, use individual `COMPASS_USE` parameters instead (see Section 27).

---

#### Step 22.3 — Environmental Light Shielding

Barometer sensors are sensitive to ultraviolet and infrared light. Calibrating or operating under direct sunlight, bright LED lighting, or near infrared sources causes the sensor to report erratic pressure readings, triggering an error lock.

**Fix:** Before calibration and before any flight, place a small piece of **open-cell black foam** directly over the barometer chip (the silver rectangle with the tiny hole). Open-cell foam allows air pressure equalization (required for barometer function) while blocking light. Closed-cell foam seals the pressure inlet and will break barometer readings entirely.

---

<a name="section-23"></a>
## Section 23 — Diagnosing ArduPilot Boot-Loops (I2C Hangs)

### Recognizing the Boot-Loop Signature

When an I2C bus conflict occurs, the QGC log displays this repeating critical error pattern:

```
[Info]     Initialising ArduPilot
[Critical] Arm: Gyros not healthy
[Critical] Arm: Baro: not healthy
[Critical] Arm: AHRS: EKF3 not started
```

This sequence then repeats indefinitely.

### Why All Three Sensors Fail Together

The IMU (Inertial Measurement Unit / Gyroscope), Barometer, and compass all communicate over the I2C or SPI bus. When an external device (like a miswired GPS) pulls one of the bus lines to ground or creates a voltage collision, the entire bus becomes unresponsive. ArduPilot queries the IMU and Barometer — neither replies — so it aborts the boot sequence and restarts, creating the infinite loop.

The EKF3 (Extended Kalman Filter) cannot start because it requires sensor input from both the IMU and Barometer. Without EKF3, the aircraft cannot compute attitude or navigation, so arming is blocked.

---

<a name="section-24"></a>
## Section 24 — Resolving Duplicate I2C Address Clashes (DPS310 Conflict)

### The Problem

Many GPS modules (including the BZGNSS BZ-251) contain an **internal DPS310 barometer**. The SpeedyBee F405 V4 also contains an **onboard DPS310 barometer**.

In the I2C protocol, every device on the bus must have a unique address. Both DPS310 chips use I2C address `0x76`. When ArduPilot queries address `0x76`, **both chips reply simultaneously**. Their data collides, the bus jams, the flight controller crashes, and the boot loop begins.

This conflict can exist even with perfect wiring and no physical shorts — it is a firmware-level address collision.

---

### Step 24.1 — The 2-Wire Hardware Isolation Test

To definitively confirm the conflict is on the I2C lines specifically (and not UART or power):

1. Keep the GPS module fully connected (5V, GND, TX, RX all connected).
2. **Desolder only the `SDA` and `SCL` wires** from the SpeedyBee.
3. Power the drone.
4. If the log reads `ArduPilot Ready` and `Barometer 1 calibration complete`, the conflict is strictly on the I2C lines — confirmed.

This test proves the GPS's UART (position data) and power connections are fine, and only the I2C barometer inside the GPS is causing the problem.

---

### Step 24.2 — Fix: Disable External Barometer Probing

In QGC Parameters:

1. Search for **`BARO_PROBE_EXT`**.
2. Set the value to **`0`**.
3. Click Write/Save.
4. Reboot the flight controller.

This single parameter change instructs ArduPilot not to probe the I2C bus for external barometers. The internal DPS310 on the SpeedyBee continues to function. The GPS's DPS310 is ignored. The address collision is eliminated.

---

<a name="section-25"></a>
## Section 25 — Resolving "Compass Not Healthy" Errors

After the barometer conflict is resolved and the drone boots to `ArduPilot Ready`, a `Compass Not Healthy` error typically remains. ArduPilot requires a complete, valid calibration matrix before it will accept any compass as healthy.

---

### Step 25.1 — Unlock Advanced Parameters

Some required compass parameters are hidden in QGC by default:

1. Click the **QGC Logo** (top left) → **Application Settings** (gear icon).
2. Under the **General** tab → **User Experience** section.
3. Set **Parameter mode** to **Advanced** (or check "Enable all parameters").
4. Return to Vehicle Setup → Parameters.

---

### Step 25.2 — Force Enable the Compass Driver

In QGC Parameters:

- Search **`COMPASS_ENABLE`** → set to `1` (enabled)
- Search **`COMPASS_AUTO_ROT`** → set to `1` (allows ArduPilot to automatically detect and correct for the GPS puck's physical orientation)
- Reboot the flight controller.

---

### Step 25.3 — Mandatory Compass Calibration

ArduPilot flags any compass as "Not Healthy" until it has received a complete valid calibration dataset. Previous I2C crashes may have written corrupted data.

1. Go to **Vehicle Setup → Sensors → Compass**.
2. Click **Clear** or **Reset** — this purges corrupted offset data from previous crash events.
3. Click **Calibrate**.
4. Perform the 6-axis physical rotation:
   - Hold the drone and rotate it slowly and smoothly through all orientations.
   - The goal is to trace a sphere in 3D space — expose every face of the compass to every direction.
   - Continue rotating until the progress bar completes.
5. Click **Save**. The `Compass Not Healthy` error will clear immediately upon successful calibration.

---

<a name="section-26"></a>
## Section 26 — Unhiding Advanced Parameters in QGC

QGC hides many ArduPilot parameters by default to simplify the interface for beginners. For advanced troubleshooting you need full access.

### Two Places to Unlock

**Method 1 — Application-level unlock:**
1. Click **QGC Logo** → **Application Settings** → **General** → **User Experience**.
2. Set **Parameter mode** to **Advanced**.

**Method 2 — Parameter screen dropdown:**
1. Go to **Vehicle Setup → Parameters**.
2. In the top-left corner, change the dropdown from **Standard** to **Full Parameter List**.

Both changes may be needed for different parameters. Apply both if a parameter you need is not appearing in search results.

---

<a name="section-27"></a>
## Section 27 — Managing "Ghost" Compasses (I2C Clutter)

### The Problem

ArduPilot supports up to 8 simultaneous compasses. The SpeedyBee F405 V3/V4 has **no internal compass**. Only the external GPS puck (Compass 1) is a real compass. If ArduPilot polls compass slots 2 through 8, it sends I2C queries to addresses that have no device — the bus receives no reply, causing timeouts that interfere with calibration.

### Fix

In QGC Parameters:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `COMPASS_USE` | `1` | Compass 1 (the GPS puck) — must be enabled |
| `COMPASS_USE2` | `0` | Disabled — no compass here |
| `COMPASS_USE3` through `COMPASS_USE8` | `0` | All disabled |

Reboot the flight controller after making changes.

> **Version note:** In ArduPilot V4.5+, the old `COMPASS_TYPEMASK` parameter is deprecated. Do not search for it. Use the individual `COMPASS_USE` toggles described above.

---

<a name="section-28"></a>
## Section 28 — Standard PreArm Failsafes

Once the I2C bus is stable and `ArduPilot Ready` is displayed, the system enforces pre-arm safety checks. All of the following must be cleared before the drone can arm.

| Error Message | Root Cause | Fix |
|--------------|-----------|-----|
| `PreArm: Battery 1 unhealthy` | FC powered by USB only (5V). ArduPilot expects flight battery voltage (e.g., 11.1V for 3S). | Plug in the LiPo battery. |
| `PreArm: Radio failsafe on` | No valid RC signal detected from the receiver. | Power on the transmitter. Verify receiver binding (solid green LED on receiver). Run Radio calibration in QGC. |
| `PreArm: Throttle below failsafe` / `Check FS_THR_VALUE` | Throttle stick PWM at zero is equal to or below the failsafe threshold. | Complete Radio Calibration first, then set `FS_THR_VALUE = 950` (see Section 35). |
| `PreArm: Logging Failed` | SD card missing, wrong format, or too slow. | See Section 43 for full SD card and logging diagnostics. |

---

<a name="section-29"></a>
## Section 29 — Resolving "Cannot Start Compass Thread"

### Error: `Cannot start compass thread`

This error appears in QGC when clicking Calibrate in the Compass section, even when `COMPASS_DEV_ID` shows a valid hardware ID (e.g., `855297`).

---

### Step 29.1 — Mandatory Prerequisite: Accelerometer Calibration First

The compass calibration thread depends on the **Extended Kalman Filter (EKF3)**. The EKF requires knowing which direction is "down" before it can start any other calibration thread. Without a completed accelerometer calibration, the EKF cannot initialize, and the compass thread cannot start.

**You must complete the 6-axis Accelerometer Calibration before attempting Compass Calibration.** (Section 9.3)

---

### Step 29.2 — The "Clean Slate" Power Sequence

A dirty boot state — where the FC was previously in an error state — can lock the calibration thread. Execute a hard reset:

1. Close QGC completely on the laptop.
2. Unplug the USB cable from the SpeedyBee.
3. Unplug the LiPo battery.
4. Plug in the **LiPo battery first** — this fully powers the GPS and compass.
5. Plug in the **USB cable**.
6. Open QGC and reattempt calibration.

The order (LiPo before USB) is important — it ensures the compass has stable power before ArduPilot starts querying it.

---

### Step 29.3 — Adjusting Calibration Fitness for Magnetic Noise

If the calibration thread starts but immediately crashes, the drone's immediate environment has too much magnetic interference for the default algorithm strictness.

In QGC Parameters:
- Search **`COMPASS_CAL_FIT`**
- Increase from default (`16`) to `32` or `48`

Higher values relax the strictness of the calibration algorithm, accepting data with more variance. This is appropriate for environments with nearby motors, ESCs, power cables, or metal structures.

---

### Step 29.4 — MAVLink Console Fallback

If the QGC graphical interface is bugged and the calibration button is unresponsive:

1. Go to **Analyze Tools → MAVLink Console**.
2. Type `magcal start` and press Enter.
3. Immediately pick up the drone and perform 6-axis rotations even if no progress bar appears.

This bypasses the GUI entirely and directly commands the ArduPilot firmware to begin calibration.

---

<a name="section-30"></a>
## Section 30 — UI-Induced Thread Crashes

### When the Hardware Is Fine but QGC Crashes the Thread

If `COMPASS_DEV_ID` shows a valid ID but QGC returns `Cannot start compass thread` after completing all prerequisites in Section 29, the QGC interface itself is sending conflicting commands.

---

### Step 30.1 — Clear Custom Autopilot Rotation

If a custom orientation matrix is selected in QGC, it attempts to apply a mathematical rotation transformation that can conflict with raw EKF data:

1. Go to **Vehicle Setup → Sensors → Compass**.
2. Find the **Autopilot Rotation** dropdown.
3. Set it to **`None`** (also shown as `Rotation_None`).

This assumes the flight controller is mounted facing forward and level. If your FC is mounted at an angle, you will need to set the correct rotation value after initial calibration is complete.

---

### Step 30.2 — Assign Sensor Priority

ArduPilot cannot start a calibration thread for a sensor that has no assigned priority:

1. In the Compass calibration screen, find the active compass (the one with "Use Compass" checked).
2. Ensure the priority dropdown explicitly reads **`Priority 1`**.
3. If it reads `Not Set`, click and select Priority 1.

---

### Step 30.3 — Disable Fast Calibration

"Fast Calibration" is designed for large aircraft (fixed-wing planes, large hexacopters) where the vehicle cannot easily be rotated by hand. It uses in-flight estimation and will break desktop calibration for standard quadcopters:

- Ensure the **Fast Calibration** checkbox is **unchecked**.

---

<a name="section-31"></a>
## Section 31 — MAVLink Console Calibration Override

### When to Use This

Use the MAVLink Console override when the QGC graphical calibration interface remains non-functional despite correct parameters, correct wiring, and correct power sequencing.

### Procedure

1. Go to **Analyze Tools → MAVLink Console**.
2. Type exactly: `magcal start 1` and press **Enter**.
   - The `1` parameter specifies calibration of only the compass assigned to Priority 1, bypassing any ghost compasses that might cause the thread to fail.
3. **Do not wait for feedback.** The console may display nothing or simply show the command disappear. This is normal — the calibration thread has started in the background.
4. Immediately pick up the drone and perform the 6-axis rotation for approximately **60 seconds**, covering all orientations as thoroughly as possible.
5. To check calibration progress:
   ```
   magcal status
   ```

---

<a name="section-32"></a>
## Section 32 — Handling "Calibration Complete" but Persistent Red Errors

### Symptom

The calibration progress bar completes and turns green. The QGC log may show `MAG0 initial yaw alignment complete`. But the Compass tab remains red and still shows `PreArm: Compass not calibrated`.

### Cause

ArduPilot caches the pre-arm failure state in RAM. It does not re-evaluate sensor health until the system is completely power-cycled — a software reboot via the QGC button is not sufficient.

### Step 32.1 — The Mandatory Hard Reboot

1. Do **not** use the software reboot button in QGC.
2. Unplug the LiPo battery.
3. Unplug the USB cable.
4. Wait **10 seconds** — this allows capacitors on the board to fully discharge and clears all in-memory state.
5. Reconnect power. The red error will clear.

---

### Step 32.2 — Disable Auto-Learning Override

If the `PreArm: Compass not calibrated` error persists after a hard reboot, ArduPilot's in-flight learning algorithm may be overriding the manual ground calibration:

In QGC Parameters:
- Search **`COMPASS_LEARN`**
- Set to **`0`** (Disabled)

This forces the EKF to use the offsets gathered during your manual calibration rotation, rather than waiting to recalculate them during flight.

---

<a name="section-33"></a>
## Section 33 — In-Flight Calibration Fallback (`COMPASS_LEARN`)

### When to Use This

Use this method only when ground calibration is completely blocked by persistent UI issues, and you have access to an open outdoor area with GPS coverage.

### Procedure

1. In QGC Parameters, set **`COMPASS_LEARN`** = **`3`**.
2. The Messages tab will confirm: `CompassLearn: Initialised`.
3. Take the drone outdoors to a location with clear sky visibility and acquire a **3D GPS lock** (typically indicated by a solid blue LED on the GPS puck or a GPS fix indicator in QGC).
4. Arm the drone in **Stabilize mode only**. Do not use Loiter or Auto — these require a healthy compass to function.
5. Fly slow, smooth circles and figure-eight patterns for **1–2 minutes**, covering a variety of headings.
6. When sufficient data is collected, ArduPilot will log: `CompassLearn: finished`.
7. `COMPASS_LEARN` automatically resets to `0`.
8. The compass error is permanently cleared.

> **Safety note:** Flying with an uncalibrated compass means GPS-assisted modes (Loiter, Auto, RTL) are unavailable and unreliable. Stabilize mode uses only the gyroscope and accelerometer for stability — it is safe to fly but does not hold position.

---

<a name="section-34"></a>
## Section 34 — Radio Integration (FlySky FS-i6 & iA6B)

### The PWM vs. Digital Protocol Distinction

**Do not connect the horizontal servo pins (CH1–CH6) on the iA6B receiver.** These are legacy PWM outputs — one wire per channel, each carrying an analog pulse signal. Modern flight controllers and ArduPilot require a **multiplexed digital signal** (i-BUS or SBUS) — all channels over a single wire, with precise digital encoding.

Using PWM pins will result in:
- Erratic control inputs
- The `Radio failsafe on` pre-arm error
- Channels appearing in QGC but with no movement or incorrect values

---

### Step 34.1 — Locate the i-BUS Port on the iA6B

The FlySky iA6B has two groups of pins:
- **Horizontal row (CH1–CH6):** Legacy PWM output. Do not use.
- **Vertical 3-pin cluster** on the far right edge of the receiver, labeled **SENS**, **i-BUS**, or **iBus servos**: This is the digital i-BUS output port.

---

### Step 34.2 — i-BUS Wiring to SpeedyBee F405

| iA6B Pin | Position | SpeedyBee F405 Pad |
|---------|----------|-------------------|
| Signal (S) | Top pin | `RX2` or `SBUS` pad |
| Power (V+) | Middle pin | `5V` pad |
| Ground (G) | Bottom pin | `GND` or `G` pad |

---

### Step 34.3 — FlySky FS-i6 Transmitter Configuration

The transmitter must be told to output the digital i-BUS signal instead of standard PPM:

1. Turn on the FS-i6 transmitter.
2. Hold the **OK** button to enter the menu system.
3. Navigate to: **System Setup → RX Setup → i-BUS Setup**.
   - On older firmware versions, this may appear as **PPM Output** — set to `On`.
4. Ensure output is set to **i-BUS**.
5. Hold **Cancel** to save and exit (the FS-i6 saves when you exit, not when you press OK).

---

<a name="section-35"></a>
## Section 35 — Throttle Failsafes & `FS_THR_VALUE`

### Why This Error Occurs

FlySky receivers output a PWM value of approximately **`1000`** when the throttle stick is at the physical zero/bottom position. ArduPilot's throttle failsafe system is designed to detect a lost radio link — when the signal drops to zero, the PWM value falls below `FS_THR_VALUE` and ArduPilot triggers emergency procedures.

If `FS_THR_VALUE` is set at or above `1000`, simply resting the throttle at zero mimics a lost radio link and puts ArduPilot in a permanent panic state.

### Fix

In QGC Parameters:
- Search **`FS_THR_VALUE`**
- Set to **`950`**

This creates a 50-point safety buffer. Normal zero throttle (≈1000) is above the threshold. Only a genuine radio power-off (which causes the PWM to drop to zero or near-zero) triggers the failsafe.

> **Note:** If `950` still triggers the failsafe, lower it further to `925` or `900`. The physical radio-off PWM value is typically near 0 or below 900, so there is plenty of margin.

> **Correction from earlier documentation versions:** Some sections listed `FS_THR_VALUE = 975`. The correct, standardized value is **`950`** as it provides a larger safety margin while remaining well above the radio-off threshold.

---

<a name="section-36"></a>
## Section 36 — Final Radio Calibration Sequence

ArduPilot will not arm until the exact PWM range of the transmitter sticks has been mapped. This teaches ArduPilot what value represents "full throttle," "zero throttle," "full left," "full right," etc.

### Procedure

1. Open QGC and go to the **Radio** tab (transmitter/remote control icon).
2. Verify the transmitter is powered on and the receiver is bound (solid green LED on the receiver).
3. **If wiring is correct:** Channel bars in QGC will be red and will move when you move the sticks.
   **If channel bars are grey and motionless:** The receiver is not sending data. Return to Section 34 and verify UART wiring and i-BUS configuration.
4. Click **Calibrate**.
5. Follow the prompts: Move both sticks to all four extreme corners (up-left, up-right, down-left, down-right). Flip all assigned switches to both positions.
6. Click **Save/Apply**.

---

<a name="section-37"></a>
## Section 37 — Final Arming Checklist

Before issuing the arm command, verify all pre-arm flags are cleared:

| Pre-Arm Error Flag | Resolution | Parameter/Action |
|--------------------|-----------|-----------------|
| `Baro: not healthy` | I2C conflict resolved | `BARO_PROBE_EXT = 0`; GPS I2C wiring corrected |
| `Compass not calibrated` | Calibration completed | Hard reboot performed after calibration (Section 32) |
| `Radio failsafe on` | iA6B wired via i-BUS | Sticks calibrated in QGC Radio tab |
| `Throttle below failsafe` | Failsafe threshold lowered | `FS_THR_VALUE = 950` |
| `Logging Failed` | SD card formatted correctly | FAT32, 32KB clusters, ≤32GB card |
| `Battery 1 unhealthy` | LiPo connected | Remove USB-only power |

### Arming the Motors

When QGC HUD displays **`ArduPilot Ready`** with no critical flags:

Pull the **throttle stick to the bottom-right corner** and hold for **3 seconds**. The motors will begin to spin at minimum idle speed.

> **Always remove propellers during bench testing.** Never arm with propellers attached unless you are prepared for the drone to fly.

---

<a name="section-38"></a>
## Section 38 — Flight Modes & Arm Switch Configuration (FlySky FS-i6)

### Step 38.1 — Map Transmitter Switches to Channels

The FS-i6 supports 6 channels. Channels 5 and 6 are used for flight modes and arming respectively.

1. Hold **OK** to enter the FS-i6 menu.
2. Navigate to **Functions Setup → Aux. channels**.
3. Set **Channel 5** to **SwC** (the 3-position switch — used for flight modes).
4. Set **Channel 6** to **SwD** (the 2-position switch — used for arming).
5. Hold **Cancel** to save.

---

### Step 38.2 — Assign Flight Modes in QGC

1. In QGC, navigate to the **Flight Modes** tab.
2. Set **Mode Channel** to **Channel 5**.
3. Toggle SwC to each position and observe which row highlights green. Assign:

| Switch Position | Mode | Description |
|----------------|------|-------------|
| SwC Up (Position 1) | `Stabilize` | Manual leveling using gyro/accelerometer. Required for first flight. No GPS dependency. |
| SwC Middle (Position 2) | `AltHold` | Automatically maintains current altitude using barometer. Pilot controls horizontal movement. |
| SwC Down (Position 3) | `Loiter` | GPS position hold. Drone holds current position and altitude. Requires healthy GPS and compass. |

---

### Step 38.3 — Assign the Arm/Disarm Switch

1. In QGC Parameters, search **`RC6_OPTION`** (Channel 6).
2. Set the value to **`153`** (ArmDisarm).
3. Write parameters and reboot.

> **Critical firmware version note:** Older ArduPilot documentation listed `RC6_OPTION = 41`. **ArduCopter V4.5+ rejects `41` as an invalid channel option.** Always use `153` for V4.5 and newer.

Flipping SwD **up** arms the drone; flipping SwD **down** immediately disarms.

---

### Step 38.4 — Verify Switch PWM Travel

ArduPilot ignores the arm command if the switch PWM does not reach the required threshold (typically ≥1800 for arm):

1. Go to **QGC → Radio** tab.
2. Flip SwD and observe **Channel 6**.
3. The bar must travel from the far left (≈1000) to the far right (≈2000).
4. **If the bar only moves halfway:** Go to the FS-i6 menu → **Functions Setup → End Points** → increase Channel 6 to **120%** on both the High and Low ends.

---

<a name="section-39"></a>
## Section 39 — Configuring ArduPilot for i-BUS (UART Parameters)

ArduPilot must be instructed to listen for RC input on the specific UART port where the iA6B is connected.

In QGC Parameters (assuming receiver is wired to `RX2`/UART2):

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | `23` | RCIN — tells ArduPilot this port receives RC input |
| `SERIAL2_BAUD` | `115` | 115200 baud (i-BUS protocol speed) |

Write parameters and reboot the flight controller.

> **If wired to a different UART port:** Substitute the correct port number (e.g., `SERIAL3_PROTOCOL`, `SERIAL3_BAUD` for UART3).

---

<a name="section-40"></a>
## Section 40 — Overriding Persistent Radio & Throttle Failsafes

### If `Radio failsafe on` Persists After Radio Calibration

FlySky transmitters sometimes output a baseline PWM value slightly variable around `1000` when the throttle is at zero. If ArduPilot's threshold is too close to this value, minor fluctuations trigger the failsafe.

In QGC Parameters:
- Set `FS_THR_VALUE` to **`950`**, `925`, or even `900`
- The failsafe will only trigger if PWM drops below this value — which only happens when the transmitter is physically powered off

Write parameters. The `Radio Failsafe Cleared` message should now persist.

---

### If `Arm: Throttle too high` Appears After Failsafe Clears

This error appears when ArduPilot does not know the precise zero-throttle position of your stick because Radio Calibration was not completed.

1. Go to **QGC → Radio** tab.
2. Click **Calibrate**.
3. Move both sticks to all extreme corners.
4. Click **Save/Apply**.

This maps the exact zero-throttle PWM, eliminating the "throttle too high" error.

---

<a name="section-41"></a>
## Section 41 — Configuring a Physical Arming Switch (Detailed)

A dedicated physical arm/disarm switch is strongly recommended for safety. In an emergency, you can instantly cut motor power by flipping a single switch, rather than executing the stick command (throttle down + yaw right, held for 3 seconds).

### Transmitter Setup

1. Enter the FS-i6 menu (hold OK).
2. Navigate to **Functions Setup → Aux. channels**.
3. Set **Channel 6** to **SwD** (2-position switch).
4. Save and Exit.

### ArduPilot Parameter Setup

In QGC Parameters:
- Search **`RC6_OPTION`**
- Set to **`153`** (ArmDisarm for ArduCopter V4.5+)
- Write parameters and reboot

> Older firmware: `RC6_OPTION = 41`. V4.5+ rejects `41` as invalid. Always use `153` on V4.5+.

### Verifying Switch PWM Range

1. QGC → **Radio** tab → flip SwD
2. Channel 6 must move from ≈1000 to ≈2000
3. If bar only moves halfway: FS-i6 → **Functions Setup → End Points** → Channel 6 → set both ends to `120%`

---

<a name="section-42"></a>
## Section 42 — Resolving Motor Rotation Failures

### Safety First

**Remove all propellers before any motor testing.** A spinning motor without a propeller is relatively harmless; a spinning motor with a propeller can cause serious injury.

---

### Step 42.1 — QGC Motor Test (Software Bypass)

This test commands motors directly, bypassing the radio and any radio/calibration errors:

1. In QGC: **Vehicle Setup → Motors**.
2. Slide the on-screen safety switch to enable motor testing.
3. Click **Test Motor** for the failing motor.

**Interpret the result:**

| Motor Test Result | Meaning | Next Step |
|------------------|---------|-----------|
| Spins normally in test | Hardware is fine — software issue | Check ESC calibration or radio mixing |
| Stutters or fails to spin | Physical hardware problem | Continue with hardware diagnostics below |

---

### Step 42.2 — Configure DShot (Recommended)

Legacy PWM ESC control requires manual throttle range calibration that can desync over time. DShot is a fully digital protocol that is immune to this:

In QGC Parameters:
- Search **`MOT_PWM_TYPE`**
- Set to **`4`** (DShot600)
- Write parameters and reboot

> **If `MOT_` parameters are missing:** See Section 45. The parameter name differs by firmware type. `SERVO_BLH_OTYPE` is the equivalent for some configurations. `Q_M_` prefix indicates QuadPlane firmware was accidentally flashed instead of ArduCopter.

---

<a name="section-43"></a>
## Section 43 — Resolving "Logging Failed" Pre-Arm Errors

### Why ArduPilot Requires Logging to Arm

ArduPilot is configured by default to refuse arming if it cannot write flight telemetry to the SD card. This is a safety feature — post-crash analysis requires flight logs.

---

### Step 43.1 — SD Card Format Requirements

The SpeedyBee F405 V4's SD card reader **cannot read exFAT partitions** and has strict format requirements:

- Card size: **32GB or smaller**
- Partition type: FAT32 (not exFAT, not NTFS, not ext4)
- Cluster size: **32KB** (64 sectors × 512 bytes per sector)

See Section 2 for complete formatting instructions.

---

### Step 43.2 — Slow SPI Bus Fix

If a valid FAT32 card is inserted but logging still fails, the flight controller is attempting to write data faster than the SD card can respond:

In QGC Parameters:
- Search **`BRD_SD_SLOWDOWN`**
- Increase from `0` to **`2`** or **`5`**
- Write parameters and reboot

Higher values reduce the SPI bus speed used to communicate with the SD card, accommodating slower cards.

---

### Step 43.3 — Bypass Logging for Bench Testing

If the SD card reader is physically damaged or you do not need flight logs during bench testing:

In QGC Parameters:
1. Search **`LOG_BITMASK`** → set to **`0`** (disables all logging)
2. Search **`ARMING_CHECK`** → click the dropdown and uncheck **Logging Available**

> **Warning:** Only do this for bench testing. Re-enable logging before actual flight — logs are essential for diagnosing any in-flight issues or crashes.

---

<a name="section-44"></a>
## Section 44 — Resolving Symmetrical Throttle Lag (Slow Motor Spin)

### Symptom

All four motors spin, but one or more accelerate noticeably slower than the others when throttle is increased. This is a calibration mismatch — the ESC does not know what signal represents "zero throttle" and "full throttle."

---

### Step 44.1 — ESC Calibration (PWM Protocol Only)

If using standard PWM (not DShot), the ESCs must be calibrated to the throttle range:

1. **Remove all propellers.**
2. Go to **QGC → Vehicle Setup → Sensors → ESC Calibration** (may also appear under the **Power** tab).
3. Follow the prompts exactly:
   - Unplug the LiPo battery.
   - Click **Start** in QGC.
   - Plug in the LiPo battery.
   - Wait for the ESC initialization tones (typically a series of beeps).
   - Move the throttle stick to zero.
4. The ESCs now know the full range from zero to maximum.

> **Tip:** If using DShot (Section 42.2), ESC calibration is not needed — DShot communicates the exact commanded value digitally, eliminating range desynchronization entirely.

---

### Step 44.2 — Increase Minimum Idle Speed

If a motor struggles to even begin rotating after arming (spins up very late or requires a significant throttle increase before moving), the minimum idle power is too low:

In QGC Parameters:
- Search **`MOT_SPIN_ARM`**
- Increase incrementally from default `0.10` to `0.15`

This parameter sets the minimum power percentage applied to all motors immediately upon arming, ensuring they overcome static friction and are ready to respond instantly.

---

<a name="section-45"></a>
## Section 45 — Unhiding Advanced Parameters in QGC (Motor Context)

### If Motor Parameters Are Not Visible

QGC's default "Standard" view hides many ArduPilot parameters:

1. Go to **Vehicle Setup → Parameters**.
2. Change the dropdown from **Standard** to **Full Parameter List** (top-left corner, below the search bar).

### Firmware-Specific Parameter Names

If `MOT_` parameters still do not appear after switching to Full Parameter List:

| Parameter to Search | Meaning |
|--------------------|---------|
| `SERVO_BLH_OTYPE` | Alternative DShot configuration parameter in some ArduPilot builds |
| `Q_M_PWM_TYPE` | Motor PWM type if **QuadPlane firmware** was accidentally flashed |

> **Identifying wrong firmware:** If all motor parameters use the `Q_M_` prefix instead of `MOT_`, QuadPlane firmware (designed for hybrid fixed-wing/multirotor aircraft) was flashed instead of ArduCopter. Reflash with the correct ArduCopter firmware for your specific SpeedyBee board version.

---

<a name="section-46"></a>
## Section 46 — Diagnosing Asymmetrical Motor Stuttering (Phase Loss)

### Symptom

One motor twitches back and forth rapidly but fails to complete a full revolution. The other three motors spin normally.

### What "Phase Loss" Means

Brushless motors work by energizing three electrical coils (phases) in sequence to create a rotating magnetic field that spins the permanent magnet rotor. All three phases must be active simultaneously. If one phase is lost (open circuit, broken wire, bad solder joint, blown MOSFET), the motor jerks toward the two remaining phases but cannot complete a rotation — this appears as rapid twitching.

**Phase loss is always a hardware failure.** It cannot be fixed by parameters, calibration, or firmware.

---

### Step 46.1 — Solder Joint Inspection

1. Inspect the **three heavy-gauge wires** connecting the stuttering motor to the ESC pads.
2. A **good joint** appears: shiny, smooth, convex, with clean wire entry.
3. A **bad (cold) joint** appears: dull, grey, grainy/crystalline texture, or balled up.
4. **Fix:** Reflow all three joints with a high-temperature soldering iron and flux. Flux cleans oxidation and helps solder flow correctly.

---

### Step 46.2 — 8-Pin FC-to-ESC Wiring Harness

On the SpeedyBee stack, motor control signals travel from the flight controller (top board) to the ESC (bottom board) through a delicate 8-pin JST or ribbon cable connector.

1. Unplug the 8-pin connector from **both** boards.
2. Use a flashlight and inspect inside the white plastic connector housing.
3. Look for:
   - Gold pins that are bent or pushed backward into the housing
   - Corrosion or green discoloration on pins
   - A pin that is pushed further back than the others (lost contact)
4. **Fix:** Carefully straighten bent pins with a fine needle. If a pin is corroded or damaged, replace the entire cable harness.

---

### Step 46.3 — Mounting Screw Short

Motor mounting screws that are too long will pass through the motor's aluminum base and pierce the copper stator windings (the internal coils). This creates a short circuit between the winding and the motor body, causing phase loss.

**Diagnostic test:**
1. Remove all four mounting screws from the suspect motor.
2. Hold the motor in place by hand and run the motor test (QGC → Motors).
3. If the motor **spins perfectly when unmounted**, the screws are too long — use shorter M3 hardware or add nylon washers.

---

### Step 46.4 — Motor Swap Test (Definitive Isolation)

To determine whether the motor itself is burned out or the ESC output is damaged:

1. **Remove all propellers.**
2. Desolder the three wires of the **suspect (stuttering) motor** from its ESC pads.
3. Desolder a **known-good motor** from a different corner.
4. Solder the **suspect motor** to the **working ESC pads**.
5. Solder the **known-good motor** to the **suspect ESC pads**.
6. Run the motor test on both.

| Result | Conclusion | Action |
|--------|-----------|--------|
| Suspect motor still stutters on working ESC pads | Motor windings are burned/broken internally | Replace the motor |
| Suspect motor now spins fine; good motor stutters on suspect pads | ESC has a blown MOSFET on that output | Replace the ESC board |

---

<a name="appendix"></a>
## Appendix — Known Inconsistencies & Corrections Log

This manual corrects all inconsistencies found across the original documentation versions. The following is a complete record of changes made and the reasoning behind each:

| Issue Found | Original State | Correction Applied |
|------------|---------------|-------------------|
| UART port numbering | Some sections referenced UART4 (pads R4/T4); later sections used UART6 (T6/R6) | Standardized throughout to **UART6** (T6/R6 pads) — this is the port used in the physical build |
| Power rule contradictions | Four different power rules appeared across versions: "USB permitted," then "USB prohibited," then "Double Power Rule," then "Single Power Rule" | Consolidated to the **Single Power Rule**: LiPo powers everything via UBEC; USB-C must be physically disconnected when UBEC is connected |
| `disable-bt` typo | One version of `config.txt` instructions showed `disable-b` (missing the `t`) | Corrected to `disable-bt` throughout, with an explicit typo warning added |
| `COMPASS_TYPEMASK` deprecated | Referenced for ArduPilot 4.5+ troubleshooting | Replaced with individual `COMPASS_USE2` through `COMPASS_USE8 = 0` toggles. `COMPASS_TYPEMASK` is deprecated in V4.5+. |
| `RC6_OPTION` arm switch value | Listed as `41` (legacy value) | Corrected to `153` for ArduCopter V4.5+. V4.5+ rejects `41` with "Invalid channel option" error. |
| `FS_THR_VALUE` inconsistency | Set to `975` in one section, `950` in another | Standardized to **`950`** as the functional tested value with adequate safety margin |
| BZGNSS BZ-251 pinout | Missing from I2C wiring sections | Full 6-pin ribbon cable pinout table added to Section 21 |
| Duplicate sections | Sections 33/41 (in-flight calibration), 34/38 (receiver wiring), and 42/43 (arm switch) were near-duplicates | Merged into single comprehensive sections with all details preserved |
| MAVProxy dependency list incomplete | Only `mavproxy` was listed for pip install | Corrected to install `mavproxy future pyserial pymavlink` together — all required on Python 3.12+ |
| `SERIAL6_OPTIONS` parameter | Not mentioned in wiring troubleshooting | Added to Section 16 — setting to `0` disables signal inversion and half-duplex modes that can cause silent hardware |
| MAVProxy launch command | Used `/dev/serial0` in some sections, `/dev/ttyAMA0` in others | Standardized to `/dev/ttyAMA0` as the direct hardware alias. `/dev/serial0` used where it is confirmed to exist as a valid symlink. |
| Accelerometer calibration prerequisite | Not clearly stated before compass calibration instructions | Section 29 now explicitly states: Accelerometer calibration must be completed before compass calibration can start. |

---

## Quick Reference — Common Parameters

| Parameter | Purpose | Typical Value |
|-----------|---------|--------------|
| `SERIAL6_PROTOCOL` | MAVLink on UART6 | `2` |
| `SERIAL6_BAUD` | Baud rate for UART6 | `921` (=921600) |
| `SERIAL6_OPTIONS` | Signal options for UART6 | `0` |
| `BRD_SER6_RTSCTS` | Hardware flow control UART6 | `0` |
| `BARO_PROBE_EXT` | External barometer probing | `0` |
| `BARO_PRIMARY` | Primary barometer selection | `0` |
| `BRD_IO_ENABLE` | Secondary I/O co-processor | `0` |
| `COMPASS_ENABLE` | Enable compass subsystem | `1` |
| `COMPASS_AUTO_ROT` | Auto-detect compass orientation | `1` |
| `COMPASS_USE` | Use Compass 1 (GPS puck) | `1` |
| `COMPASS_USE2`–`COMPASS_USE8` | Ghost compasses disabled | `0` |
| `COMPASS_LEARN` | In-flight calibration mode | `0` (normal); `3` (in-flight learn) |
| `COMPASS_CAL_FIT` | Calibration strictness | `16` (default); `32`–`48` (noisy environment) |
| `FS_THR_VALUE` | Throttle failsafe threshold | `950` |
| `SERIAL2_PROTOCOL` | Protocol for UART2 (RC input) | `23` (RCIN) |
| `SERIAL2_BAUD` | Baud rate for UART2 (RC input) | `115` (=115200) |
| `RC6_OPTION` | Function for Channel 6 switch | `153` (ArmDisarm, V4.5+) |
| `MOT_PWM_TYPE` | ESC protocol | `4` (DShot600) |
| `MOT_SPIN_ARM` | Minimum idle power on arm | `0.10`–`0.15` |
| `BRD_SD_SLOWDOWN` | SD card write speed reduction | `0` (fast card); `2`–`5` (slow card) |
| `LOG_BITMASK` | Logging channels bitmask | `0` (disabled, bench only) |

---

*End of complete documentation. All sections from both source documents have been merged, inconsistencies resolved, and information presented for readers with no prior context on the topic.*
