
# Drone Swarm P2P Network: Infrastructure & Debugging SOP

**Target OS:** Ubuntu 24.04 LTS Server (Raspberry Pi 4)
**Objective:** Establish a headless, bidirectional, fault-tolerant network for the drone swarm, with automated fallback for developer debugging.

---


## 1. Remote Access & SSH Connectivity

To configure or debug the drones, you must establish an SSH tunnel.

### 1.1 Connecting via Wi-Fi (The Standard Route)

If the drone and your PC are on the same local network (like a shared mobile hotspot):

**Method A: The Zero-IP Method (mDNS)**
Ubuntu natively broadcasts its hostname to the local network. This is the fastest method.

```bash
ssh drone1@drone1.local

```

**Method B: The Explicit IP Method**
If `.local` fails to resolve, you must use the explicit IP.

1. **Find the IP Address:**
* If using a mobile hotspot, check the "Connected Devices" section in your phone's settings.
* If you have a monitor plugged into the Pi, run: `hostname -I`
* To scan the local network from a Linux PC: `sudo nmap -sn 10.42.0.0/24` (Replace with your actual subnet).


2. **Establish the SSH Connection:**
```bash
ssh drone1@<INSERT_IP_ADDRESS>

```



### 1.2 The Legacy RSA Key Error

Ubuntu 24.04 strictly enforces modern cryptography. If you attempt to SSH into a newly flashed Pi and receive the error:
`Unable to negotiate with <IP> port 22: no matching host key type found. Their offer: ssh-rsa`

**The Fix:** Force the SSH client to accept the older RSA algorithm:

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa drone1@<INSERT_IP_ADDRESS>

```

### 1.3 The Zero-Config Emergency Backdoor (IPv6 Link-Local)

**The Concept:** If static IPv4 configurations fail, bypass NetworkManager entirely. The kernel automatically assigns an IPv6 Link-Local address to the physical hardware the moment an Ethernet cable connects.

**Execution:**

1. **Find your PC's Ethernet interface name:** Run `ip link` on your laptop (look for names like `enp8s0`, `eth0`, or `eno1`).
2. **Ping the local multicast node** to force the Pi to announce its hardware address over the physical cable (replace `enp8s0` with your actual interface name):
```bash
ping -6 -I enp8s0 ff02::1

```


3. **Extract the Address:** Look for the replies starting with `fe80::` that do not belong to your PC. This is the drone.
4. **SSH via Hardware Scope:** You must append your laptop's interface name (`%enp8s0`) to the end of the IP so the SSH client knows which physical cable to push the data through:
```bash
ssh drone1@fe80::<INSERT_PI_ADDRESS_HERE>%enp8s0

```
---

## 2. The Ethernet Lifeline (Static Hardwire)

**The Problem:** When configuring Wi-Fi settings via an active Wi-Fi SSH session, applying changes will instantly sever your connection, locking you out of the drone. Furthermore, simply plugging an Ethernet cable between a PC and a Pi fails because there is no router to assign IP addresses (DHCP timeout), causing the port to shut down automatically.

**The Solution:** We enforce a static Point-to-Point (P2P) Ethernet connection. This creates a bulletproof "backdoor" that never drops, regardless of what the Wi-Fi chip is doing.

### Note : 
This network is already created on drone1 as of Mar 5, 26.

### 2.1 Execution (Run on Developer PC)

Set the laptop's Ethernet port (`enp8s0` or similar) to a static IP:

```bash
sudo nmcli connection add type ethernet ifname enp8s0 con-name "PC-Static" ipv4.method manual ipv4.addresses 10.10.2.2/24
sudo nmcli connection up "PC-Static"

```

### 2.2 Execution (Run on Drone via Keyboard/Monitor or Initial Wi-Fi)

Set the Pi's Ethernet port (`eth0`) to the adjoining static IP:

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name "Pi-Static" ipv4.method manual ipv4.addresses 10.10.2.1/24
sudo nmcli connection up "Pi-Static"

```

**Result:** You can now permanently SSH into the drone using `ssh drone1@10.10.2.1` via the physical cable.

---

## 3. The `wlan0` Liberation (Fixing the "Unavailable" State)

**The Problem:** Out of the box, Ubuntu 24.04 Server uses `systemd-networkd` combined with `Netplan` to manage Wi-Fi. However, for dynamic drone swarm environments (switching between Hotspot and Client modes), we require `NetworkManager`. When both engines try to control the `wlan0` hardware simultaneously, the kernel locks the chip in an `unavailable` state. `nmcli device wifi rescan` will fail with "Scanning not allowed."

### 3.1 The Software Brain Transplant

To hand total control of the hardware to NetworkManager, we must install the missing backend, strip `networkd`'s privileges, and rewrite the Netplan configuration.

```bash
# 1. Install the backend Wi-Fi engine
sudo apt update && sudo apt install wpasupplicant -y
sudo systemctl enable --now wpa_supplicant

# 2. Overwrite Netplan to strictly use NetworkManager (Removes hardcoded 'wifis:' blocks)
sudo bash -c 'cat > /etc/netplan/50-cloud-init.yaml <<EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      optional: true
      dhcp4: true
EOF'

# 3. Apply the changes and reboot the manager
sudo netplan apply
sudo systemctl restart NetworkManager

```
### Previous Code inside of 50-cloud-init.yaml file : 
```
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      regulatory-domain: "IN"
      access-points:
        "Palash":
          auth:
            key-management: "psk"
            password: "36d9e8e87138d782816e06988d1731a901e734e94105e25a8c27fdc326bd619a"
```

### 3.2 The Hardware Frequency Constraint

If `wlan0` is awake but fails to find your mobile hotspot (e.g., `Error: No network with SSID found`), it is a physics limitation. The Raspberry Pi 4 driver often rejects 5GHz bands on certain channels due to regional firmware locks.
**The Fix:** Force your mobile phone hotspot to broadcast on the **2.4 GHz band** (often labeled "Extend Compatibility" on Android/iOS).

---

## 4. Network Automation: The Priority Waterfall

**The Objective:** The drone must be autonomous. It needs to know when to connect to a developer for coding, and when to act as a swarm node, without requiring manual command injection.

**The Logic:** NetworkManager uses an integer-based `autoconnect-priority` system. Higher integers are executed first.

### 4.1 Step-by-Step Execution: Building the Hierarchy

You must physically create and save the developer Wi-Fi profile in the drone's database *before* you can assign it a priority.

**Step 1: Broadcast & Connect to Developer Wi-Fi**

1. Turn on your mobile hotspot (e.g., named `DevDebug`). Ensure it is set to the 2.4 GHz band.
2. Force the drone to scan the airspace:
```bash
sudo nmcli device wifi rescan && sleep 3 && nmcli device wifi list

```


3. Connect the drone to the hotspot to permanently save the credentials:
```bash
sudo nmcli device wifi connect "DevDebug" password "INSERT_HOTSPOT_PASSWORD_HERE"

```



**Step 2: Assign Tier 1 Priority (Developer Mode)**
Make the `DevDebug` network the highest priority (Tier 1). On boot, the drone will always hunt for this first.

```bash
sudo nmcli connection modify "DevDebug" connection.autoconnect-priority 200 connection.autoconnect yes

```

**Step 3: Assign Tier 2 Priority (Swarm Mode)**
Make the `MasterDrone` hotspot the fallback (Tier 2). If `DevDebug` is turned off or out of range, the drone gives up and broadcasts the swarm network.

```bash
sudo nmcli connection modify "MasterDrone" connection.autoconnect-priority 100 connection.autoconnect yes

```

### 4.2 The Safety Catch

If the drone is actively broadcasting `MasterDrone` and flying, and a developer walks by with the `DevDebug` hotspot turned on, the drone **will not** drop the active swarm connection to switch to Tier 1. It maintains state to prevent mid-air communication failure. To switch back to Tier 1, the drone must be manually rebooted or commanded via the Ethernet lifeline.

---

Your documentation is now rigorous and complete. Stop tweaking the text formatting.

Are we immediately transitioning to the C++ bidirectional rolling queue logic for real-time telemetry, or are you pivoting to the Universal Motor design?
