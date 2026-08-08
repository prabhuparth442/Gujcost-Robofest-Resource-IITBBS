> **Historical note:** This document was written when the project used a Pixhawk + PX4.
> The current setup uses a **SpeedyBee F405 running ArduCopter**. SITL commands
> and FC-specific parameters in this doc apply to ArduCopter SITL, not PX4 SITL.

# 📘 Complete Technical Documentation: Raspberry Pi 4 Drone Companion Computer Setup

**Project:** Autonomous Drone Control System for GUJCOST Competition  
**Hardware:** Raspberry Pi 4 Model B + Pixhawk Flight Controller  
**Operating System:** Ubuntu Server 24.04 LTS (64-bit)  
**Objective:** Create a headless, wireless-enabled companion computer capable of autonomous flight control via MAVSDK

---

## Table of Contents

1. [Phase 1: Connectivity & OS Setup](#phase-1-connectivity--os-setup)
2. [Phase 2: Wi-Fi Optimization](#phase-2-wi-fi-optimization)
3. [Phase 3: Drone Control Software](#phase-3-drone-control-software)
4. [Phase 4: Hardware & Telemetry](#phase-4-hardware--telemetry)
5. [Critical Warnings & Master Checklist](#critical-warnings--master-checklist)

---

# Phase 1: Connectivity & OS Setup

## Section 1: Initial Headless Connection Troubleshooting

### 1.1 Hardware & Power Diagnosis

**Error Symptom:**
- Solid Red LED (power indicator ON)
- Green LED stopped blinking (boot completed)
- Device remained offline, unable to connect to network

**Root Cause:**
- **Voltage Sag:** Standard phone chargers fail to provide stable 5.1V / 3.0A required by Pi 4
- **Brownout Mechanism:** Wi-Fi chip power-up creates momentary spike; phone charger voltage dips, resetting Wi-Fi chip
- **Cable Quality:** Thin USB cables increase resistance, exacerbating undervoltage

**The Fix:**
- **Immediate:** Switch to Official Raspberry Pi power supply or high-quality 3A adapter
- **Long Term:** Use dedicated 5V/3A UBEC (Battery Elimination Circuit) powered by LiPo battery for flight operations

### 1.2 Wi-Fi Compatibility Checklist

**Mobile Hotspot Configuration:**
- **Frequency Band:** Force 2.4GHz (Enable "Maximize Compatibility" on iOS or "AP Band" on Android)
- **Security Protocol:** Downgrade from WPA3 to WPA2 Personal (AES)
- **SSID Formatting:** Remove special characters, spaces, and emojis (e.g., rename to "Palash")
- **Hidden Hotspot Trick:** Toggle hotspot OFF/ON immediately before powering Pi to ensure active broadcast

### 1.3 Legacy Configuration Attempt (wpa_supplicant)

**Error Symptom:**
- Pi booted (Green LED stopped blinking) but did not connect to network
- Used `wpa_supplicant.conf` method from previous DroneDojo projects

**Root Cause:**
- **OS Architecture Mismatch:** Ubuntu 24.04 uses Netplan and Cloud-Init for networking
- **Legacy Incompatibility:** Ubuntu 24.04 completely ignores `wpa_supplicant.conf` files

**Configuration Paradigm Shift:**

| Feature | DroneDojo (RasPi OS) | Ubuntu 24.04 LTS |
|---------|---------------------|------------------|
| Config File | wpa_supplicant.conf | network-config |
| Location | /boot/ | /system-boot/ |
| Format | Text / Script | YAML (Strict Indentation) |

---

## Section 2: Operating System Migration

### 2.1 The "Missing Engine" Diagnosis

**Error Symptom:**
- SSH configuration files appeared empty or missing
- Connection attempts failed with "Connection refused"

**Root Cause:**
- **Wrong OS Variant:** Ubuntu Desktop was flashed instead of Ubuntu Server
- Ubuntu Desktop does NOT include `openssh-server` by default
- Without this software "engine," no SSH access is possible

**The Fix:**
- Re-flash SD card with **Ubuntu Server 24.04 LTS (64-bit)**
- Server version is lighter, headless-ready, and includes SSH by default

### 2.2 The "Golden Config" (Raspberry Pi Imager Settings)

**Configuration Parameters Applied:**

```yaml
Operating System: Ubuntu Server 24.04 LTS (64-bit)
Hostname: drone1
SSH Interface: Enabled with "Use password authentication"
User Credentials:
  Username: ubuntu
  Password: ubuntu (change on first login)
Wireless LAN:
  SSID: Palash
  Password: palash2006
  Country Code: IN (India)
Locale Settings:
  Time zone: Asia/Kolkata
```

### 2.3 First Boot Protocol

**The Wait Time:**
- Strict wait period of **3 to 5 minutes** after power-on
- Allows cloud-init to resize filesystem and generate unique SSH keys
- **Premature Connection Risk:** "Connection Refused" error if attempted too early

### 2.4 The user-data Configuration (Avoiding Login Loops)

**Critical Setting:**

```yaml
#cloud-config
hostname: ubuntu-pi
ssh_pwauth: true
users:
  - name: ubuntu
    gecos: Ubuntu User
    sudo: ALL=(ALL) NOPASSWD:ALL
    groups: users,admin
    shell: /bin/bash
    lock_passwd: false
    passwd: "$6$rounds=4096$ubuntu$8.2/S.n.q./g.u.n.n.y."
chpasswd:
  expire: false  # <-- CRITICAL: Prevents forced password change prompt
```

---

## Section 3: Network Configuration (Netplan vs. Cloud-Init)

### 3.1 The "Source of Truth" (network-config)

**Corrected YAML Configuration:**

```yaml
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "Palash":
        password: "palash2006"
```

**Critical Syntax Rules:**
- Indentation: Must use 2 spaces per level (NO TABS)
- Quoting: SSID and Password strings must be wrapped in double quotes
- Structure: `wifis:` block must be vertically aligned with `ethernets:`

### 3.2 The "Nuclear Option" (Disable Cloud-Init Networking)

**Issue:** cloud-init overwrites manual Wi-Fi changes on every boot

**Resolution:**

```bash
sudo bash -c 'echo "network: {config: disabled}" > \
/media/nonu/writable/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg'
```

**Effect:** Forces cloud-init to ignore network setup, handing control to manual Netplan files

### 3.3 The Ethernet Bridge Bypass (Emergency Access)

**Ethernet Recovery Procedure:**

1. **Laptop Configuration ("Shared" Mode):**
   - Settings > Network > Wired > IPv4
   - Change IPv4 Method to "Shared to other computers"
   - This activates DHCP server on laptop's Ethernet port

2. **Identifying the New IP Range:**
   ```bash
   hostname -I  # On laptop
   # Output: 10.42.0.1
   # Conclusion: Pi will be on 10.42.0.x range
   ```

3. **Troubleshooting "Silent" Pi:**
   - **Root Cause:** `99-disable-network-config.cfg` prevented IP request even over Ethernet
   
4. **The Fix:**
   ```bash
   # Remove blocker
   sudo rm /media/nonu/writable/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
   
   # Clear Cloud-Init cache
   sudo rm -rf /media/nonu/writable/var/lib/cloud/*
   ```

5. **Successful Discovery:**
   ```bash
   sudo nmap -sn 10.42.0.0/24
   # Result: Pi found at 10.42.0.20
   
   ssh ubuntu@10.42.0.20
   ```

### 3.4 Advanced Network Discovery (Subnet Deduction)

**The Logic Chain:**

1. **Self-Identification:**
   ```bash
   hostname -I  # On laptop
   # Output: 10.14.28.128
   ```

2. **Deduction:** Laptop and Pi share same subnet (10.14.28.x)

3. **Targeted Scanning:**
   ```bash
   sudo nmap -sn 10.14.28.0/24
   ```

**Key Lesson:** Never assume subnet is 192.168.1.x. Always check host's IP first.

### 3.5 The Permanent Wi-Fi Fix

**3-Step "Permanent" Fix:**

1. **Edit Boot Partition (Source of Truth):**
   - Edit `network-config` in `system-boot` partition
   - This is the template Cloud-Init uses

2. **Cache Clear Command:**
   ```bash
   sudo rm -rf /media/nonu/writable/var/lib/cloud/*
   ```
   - Deletes "I have already run" flag
   - Forces Cloud-Init to run as if first boot

3. **Disable File Cleanup:**
   - Delete `99-disable-network-config.cfg` 
   - Allow Cloud-Init to run using new, correct source file

---

## Section 4: SSH Configuration & Debugging

### 4.1 Resolving "Connection Refused" & "Connection Reset"

**Error Messages:**
```
ssh: connect to host ... port 22: Connection refused
kex_exchange_identification: read: Connection reset by peer
```

**Root Cause:**
- **Cloud-Init Latency:** First boot generates unique cryptographic SSH host keys
- **Service Unavailability:** SSH daemon cannot accept connections until key generation completes

**Resolution:**
- **The 5-Minute Rule:** Wait 3-5 minutes after power-on before attempting connection

### 4.2 Resolving "Host Key Verification Failed"

**Error Message:**
```
WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!
IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY!
```

**Root Cause:**
- **Fingerprint Mismatch:** Laptop remembers old OS fingerprint
- **New Identity:** Re-flashed SD card created new cryptographic fingerprint

**Resolution:**
```bash
ssh-keygen -f "/home/nonu/.ssh/known_hosts" -R "drone1.local"
# Also run for IP address if using: -R "10.42.0.20"
```

### 4.3 Manual Password Reset ("Hacker Method")

**Procedure:**

1. **Generate Password Hash:**
   ```bash
   openssl passwd -6 "drone123"
   # Output: $6$....
   ```

2. **Edit Shadow File:**
   ```bash
   sudo nano /media/nonu/writable/etc/shadow
   # Locate line starting with ubuntu:
   # Replace hash between first and second colons
   ```

### 4.4 Final Successful Connection

```bash
ssh ubuntu@drone1.local
# Verification: Prompt changes to ubuntu@drone1:~$
```

---

# Phase 2: Wi-Fi Optimization

## Section 5: Unlocking 5GHz Wi-Fi (Regulatory Domains)

### 5.1 Diagnosis: The "World Mode" Lock

**Symptom:** Pi connected fine on 2.4GHz but failed on 5GHz

**Root Cause:**
- Ubuntu Server defaults to "00" (World) Regulatory Domain
- Disables all 5GHz channels globally until country code is set
- Prevents violating local radio laws

**Verification:**
```bash
iw reg get
# Output: country 00: (World Regulatory Domain)
```

### 5.2 The "Unable to Locate Package" Error (DNS Fix)

**Error:** `E: Unable to locate package crda`

**Causes:**
1. "Universe" repository disabled by default
2. DNS failure (Pi has IP but cannot resolve domain names)

**Resolution:**

```bash
# Fix DNS
sudo nano /etc/resolv.conf
# Add: nameserver 8.8.8.8

# Verify internet
ping -c 3 google.com

# Enable repository
sudo add-apt-repository universe

# Update package list
sudo apt update
```

### 5.3 Installing Regulatory Tools

**Issue:** `crda` package is deprecated in Ubuntu 24.04

**Solution:**
```bash
sudo apt install iw wireless-regdb -y
```

### 5.4 Forcing the "IN" (India) Domain

**Immediate Set:**
```bash
sudo iw reg set IN
```

**Verification:**
```bash
iw reg get
# Output: country IN: DFS-ETSI
```

**Permanent Fix (Persistence):**
```bash
crontab -e
# Add entry:
@reboot sudo iw reg set IN
```

### 5.5 Fixing the "Race Condition"

**Problem:** Pi connects to 2.4GHz before "India" command runs on boot

**Final Fix:** Edit Netplan file

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

```yaml
wifis:
  wlan0:
    regulatory-domain: "IN"
    dhcp4: true
    optional: true
    access-points:
      "Palash":
        password: "palash2006"
```

This ensures correct rules apply during Wi-Fi interface initialization.

---

## Section 6: Wi-Fi Power Management & Stability

### 6.1 Diagnosis: Power Saving Mode

**Symptom:**
- Pi connects, responds to few pings, then drops packets
- "Destination Host Unreachable" despite having IP address

**Root Cause:**
- Aggressive "Power Save" mode enabled by default
- On 5GHz, sleep mode causes chip to miss router beacon frames

### 6.2 The Fix: Disabling Power Save

**Immediate Command:**
```bash
sudo iw dev wlan0 set power_save off
```

**Verification:**
```bash
iw dev wlan0 get power_save
# Output: Power save: off
```

### 6.3 Persistence via Crontab

```bash
crontab -e
# Add entry:
@reboot sleep 10 && sudo iw dev wlan0 set power_save off
```

**Note:** `sleep 10` ensures Wi-Fi interface is fully up before command executes

### 6.4 The "Drone Reality" Decision (2.4GHz vs 5GHz)

**5GHz:**
- Higher speed (video streaming)
- Lower range
- Poor penetration through carbon fiber frames
- High risk of disconnects at range

**2.4GHz:**
- Better range and penetration
- More stable for critical command/control telemetry
- Recommended for flight safety

**Final Verdict:** Stick to 2.4GHz for competition, keep power save OFF to minimize latency

---

# Phase 3: Drone Control Software

## Section 7: MAVSDK & Python Environment

### 7.1 MAVSDK Installation (Breaking System Packages)

**Constraint:** Ubuntu 24.04 blocks global pip installs (PEP 668)

**Resolution:**
```bash
pip3 install mavsdk --break-system-packages
```

### 7.2 USB Permission Fix (dialout)

**Issue:** Script failed to connect to Pixhawk via USB

**Root Cause:** User `ubuntu` lacks permission to access serial ports

**Fix:**
```bash
sudo usermod -a -G dialout $USER
# CRITICAL: Logout and log back in (or reboot)
```

### 7.3 Identifying the Connection Port

**Method:**
```bash
# Connect Pixhawk via USB
ls /dev/ttyACM*
# Result: /dev/ttyACM0
```

**Connection String:**
```python
serial:///dev/ttyACM0:115200
```

### 7.4 The asyncio Event Loop Warning

**Error:** `DeprecationWarning: There is no current event loop`

**Root Cause:** Older scripts used deprecated `asyncio.get_event_loop()`

**The Fix:**

```python
# OLD (Deprecated)
loop = asyncio.get_event_loop()
loop.run_until_complete(run())

# NEW (Python 3.12+)
if __name__ == "__main__":
    asyncio.run(run())
```

---

## Section 8: Safe Indoor Testing (Scripting)

### 8.1 The "Indoor Test" Script (Arm Only)

**Purpose:** Test motors indoors without GPS lock requirement

**Key Script Features:**

```python
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    
    # Battery check with timeout
    try:
        async with asyncio.timeout(3):
            async for battery in drone.telemetry.battery():
                print(f"Battery: {battery.voltage_v}V")
                break
    except asyncio.TimeoutError:
        print("Battery telemetry timeout - continuing anyway")
    
    # Arm motors
    await drone.action.arm()
    print("Motors armed - spinning at idle")
    
    # Let motors spin
    await asyncio.sleep(5)
    
    # Disarm
    await drone.action.disarm()
    print("Motors disarmed")

if __name__ == "__main__":
    asyncio.run(run())
```

### 8.2 Bypassing Safety Checks

**Error:** "Pre-arm check: GPS Lock"

**Solutions:**
1. **QGroundControl:** Disable specific "Arming Checks" (GPS)
2. **Hardware:** Manually switch RC transmitter to STABILIZED or ALTITUDE HOLD mode before running script

---

## Section 9: Simulation Environment (Gazebo/SITL)

### 9.1 Installing PX4 SITL (Software In The Loop)

**Platform:** Acer Predator Laptop (Ubuntu)

**Installation:**
```bash
# Clone PX4 Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Run setup script
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Launch simulation
cd PX4-Autopilot
make px4_sitl gz_x500
```

**Result:** 3D Gazebo window opens with virtual drone

### 9.2 Connecting Python to Simulation

**Connection String Change:**

```python
# Real Drone
await drone.connect(system_address="serial:///dev/ttyACM0:115200")

# Simulation
await drone.connect(system_address="udp://:14540")
```

**Validation:** Script successfully commanded virtual drone to takeoff, hover for 10 seconds, and land

---

## Section 10: MAVProxy Installation & Environment Issues

### 10.1 The pipx "Sealed Box" Error

**Error:** `ModuleNotFoundError: No module named 'future'`

**Root Cause:**
- pipx creates isolated virtual environment for each application
- Even globally installed packages are invisible inside pipx environment

**The Fix:**
```bash
pipx inject mavproxy future pyserial
```

---

# Phase 4: Hardware & Telemetry

## Section 11: Power Hardware Diagnosis (Brownouts)

### 11.1 The "Red LED" Diagnostic

**Symptom:** Red LED turning off intermittently

**Meaning:**
- Red LED hardwired to voltage supervisor
- Turns OFF if 5V rail drops below 4.63V
- Confirms "Brownout" condition

### 11.2 Root Cause: Regulator Failure

**Hardware Setup:** LM2596 buck converter

**Failure Mode:**
- Cheap LM2596 modules fail to deliver rated 3A
- Pi spikes to 2-3A load → voltage sags to ~4.5V
- Triggers Red LED warning and risks reboot loop

### 11.3 The "5.25V" Trick & Capacitor Fix

**Stabilization Strategy:**

1. **Over-Voltage Headroom:**
   - Tune regulator output to 5.25V (instead of 5.0V)
   - Provides 0.25V buffer
   - Under load, drops to ~5.0V instead of 4.5V

2. **Capacitor Buffer:**
   - Solder 470µF or 1000µF capacitor across Pi's 5V/GND GPIO pins
   - Acts as temporary energy reservoir for spikes

3. **Wire Gauge:**
   - Replace thin jumper wires with soldered, thick-gauge wires
   - Reduces resistance

**Long-Term Solution:** Replace LM2596 with high-quality 3A UBEC for flight operations

---

## Section 12: The MAVProxy Bridge

### 12.1 Establishing the Bridge

**Purpose:** Route MAVLink data from USB to laptop over Wi-Fi

**Command:**
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600 \
--out udp:10.14.28.128:14550
```

**Parameters:**
- `--master`: USB port connecting Pi to Pixhawk
- `--out`: Target laptop IP and port (14550 is standard for QGroundControl)

### 12.2 Verifying the Link

**Result:** QGroundControl on laptop automatically detected stream and connected, displaying live telemetry

---

## Section 13: Battery Monitoring Config (Fixing "0V")

### 13.1 Isolating the Issue

**Symptom:**
- MAVSDK script hanging at "Fetching battery status..."
- QGroundControl showing red battery icon with 0.0V

**Test:** Connected Pixhawk to QGroundControl via MAVProxy bridge

**Result:** Confirmed Pixhawk configuration error, not script error

### 13.2 Configuration Fixes

**QGroundControl Settings:**

Navigate to: **Vehicle Setup > Power**

1. **Sensor Type:** Change from "Power Module 90A" to "Other"

2. **Magic Numbers (Calibration):**
   - Voltage Divider: **18.2** (Standard for generic yellow XT60 power modules)
   - Amps per Volt: **18.02**

3. **Port Assignment:**
   - Verify 6-pin cable in POWER1
   - Set "Voltage Pin" dropdown to **Pixhawk..._PM1**

### 13.3 Success Confirmation

**Result:**
- After applying divider value of 18.2 and rebooting vehicle
- Voltage reading jumped to 12.20V
- Script successfully passed battery check and armed motors

---

# Critical Warnings & Master Checklist

## 1. Critical Safety Warnings

### ⚠️ The "Zombie" Disable File

**Risk:** `99-disable-network-config.cfg` file disables ALL networking

**Danger:**
- If accidentally recreated or left on SD card
- Permanently disables Wi-Fi AND Ethernet on next boot
- Complete network lockout

**Action:**
```bash
sudo rm /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```

### ⚠️ The "Fake" Regulator Trap (LM2596)

**Risk:** Cheap LM2596 clones rated for 3A fail at 1.5A

**Dangers:**
- Raspberry Pi 4 draws current spikes of ~3A
- Clone regulator overheats and cuts power mid-flight (Brownout)
- Drone crashes

**Voltage Scaling Danger:**
- 4S battery (16.8V) or 6S battery (25.2V) to 5V generates massive heat
- LM2596 will likely fail thermally under this load

**MANDATORY Action:** Upgrade to high-quality 3A UBEC before any outdoor flight

### ⚠️ Simulation Command Confusion

**Risk:** Running SITL/Gazebo on Raspberry Pi

**Danger:**
- Gazebo requires 3D graphics acceleration and significant RAM
- Will crash Pi OS

**Rule:**
- **Pi** = Control Scripts Only
- **Laptop** = Simulation

---

## 2. Technical "Gotchas" & Solutions

### 🔧 Python 3.12+ Asyncio Fix

**Modern Entry Point:**
```python
if __name__ == "__main__":
    asyncio.run(run())
```

### 🔧 MAVProxy "Sealed Box" Error

**Solution:**
```bash
pipx inject mavproxy future pyserial
```

### 🔧 Indoor Flight Strategy (No GPS)

**Workaround:**

1. **Code:** Use `drone.action.arm()` only (spins motors at idle)
2. **Hardware:** Manually switch RC transmitter to STABILIZED or ALTITUDE HOLD mode before running script

---

## 3. Pre-Flight Master Checklist

### Network Configuration
- [ ] Hotspot set to 2.4GHz
- [ ] Hotspot security set to WPA2 Personal
- [ ] Wi-Fi power save disabled: `iw dev wlan0 get power_save` shows "off"
- [ ] Regulatory domain set to IN: `iw reg get` shows "country IN"
- [ ] No `99-disable-network-config.cfg` file exists
- [ ] Can ping Pi from laptop successfully

### Power System
- [ ] Using high-quality 3A UBEC (not LM2596)
- [ ] Voltage output tuned to 5.25V
- [ ] Thick-gauge wires soldered (not jumper wires)
- [ ] Red LED stays solid during boot and operation
- [ ] 470µF+ capacitor installed across 5V/GND

### Software Environment
- [ ] MAVSDK installed: `pip3 list | grep mavsdk`
- [ ] User in dialout group: `groups` shows dialout
- [ ] Pixhawk connection verified: `ls /dev/ttyACM*` shows device
- [ ] Scripts use `asyncio.run()` (not deprecated event loop)

### Pixhawk Configuration
- [ ] Battery monitoring shows correct voltage (not 0V)
- [ ] Voltage divider set to 18.2
- [ ] Power module type set to "Other"
- [ ] GPS arming check disabled (for indoor testing)
- [ ] RC transmitter paired and functional

### Safety Protocols
- [ ] Propellers removed for initial code testing
- [ ] Indoor tests use arm-only script (no takeoff command)
- [ ] Simulation validated on laptop before real flight
- [ ] Emergency kill switch on RC transmitter tested
- [ ] Clear flight area with no obstacles

---

## 4. Key Lessons Learned

1. **Power Quality is Critical:** Phone chargers and cheap buck converters are insufficient for stable Raspberry Pi operation

2. **OS Choice Matters:** Ubuntu Desktop vs. Server variants have fundamentally different networking and service configurations

3. **Cloud-Init vs. Manual Config:** Understanding "source of truth" hierarchy prevents configuration conflicts

4. **Regulatory Domains:** 5GHz Wi-Fi requires explicit country code configuration due to legal frequency restrictions

5. **Power Management:** Disabling Wi-Fi power save is essential for real-time telemetry applications

6. **Safety First:** Indoor testing scripts and simulation environments prevent costly crashes during development

7. **Calibration Requirements:** Flight controller power modules require specific voltage divider values for accurate battery monitoring

---

## 5. Common Error Messages & Solutions

| Error Message | Root Cause | Solution |
|--------------|------------|----------|
| `Connection refused` | SSH service not started yet | Wait 3-5 minutes after boot |
| `Host key verification failed` | OS re-flashed, new fingerprint | `ssh-keygen -R "drone1.local"` |
| `Unable to locate package` | Universe repository disabled | `sudo add-apt-repository universe` |
| `ModuleNotFoundError: future` | pipx isolated environment | `pipx inject mavproxy future pyserial` |
| `Pre-arm check: GPS Lock` | Indoor flight without GPS | Disable GPS check in QGroundControl |
| Battery shows 0.0V | Wrong power module preset | Set voltage divider to 18.2 |
| Red LED turns off | Voltage brownout | Tune regulator to 5.25V, add capacitor |
| Wi-Fi drops packets | Power save mode enabled | `sudo iw dev wlan0 set power_save off` |

---

## 6. Emergency Recovery Procedures

### Lost SSH Access
```bash
# 1. Connect Pi to laptop via Ethernet
# 2. On laptop, set wired connection to "Shared to other computers"
# 3. Find Pi IP
sudo nmap -sn 10.42.0.0/24
# 4. Connect
ssh ubuntu@10.42.0.XX
```

### Forgotten Password
```bash
# 1. Insert SD card in laptop
# 2. Generate new hash
openssl passwd -6 "newpassword"
# 3. Edit shadow file
sudo nano /media/nonu/writable/etc/shadow
# 4. Replace hash for ubuntu user
```

### Network Configuration Broken
```bash
# 1. Insert SD card in laptop
# 2. Edit source of truth
sudo nano /media/nonu/system-boot/network-config
# 3. Clear Cloud-Init cache
sudo rm -rf /media/nonu/writable/var/lib/cloud/*
# 4. Reboot Pi
```

---

## System is Now Flight-Ready

All critical subsystems validated:
- ✅ Connectivity (headless Wi-Fi)
- ✅ Power delivery (stable voltage)
- ✅ Control software (MAVSDK + Python)
- ✅ Telemetry (wireless MAVProxy bridge)
- ✅ Safety protocols (indoor testing, simulation)

**Ready for GUJCOST Drone Competition**
