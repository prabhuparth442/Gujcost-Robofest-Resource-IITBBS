# 🚁 Raspberry Pi Swarm P2P Network: Setup & Troubleshooting

## 1\. Overview & System Architecture

### 1.1 Project Goal

Establish a **headless, bidirectional Wi-Fi connection** between two Raspberry Pi 4s running Ubuntu 24.04 LTS Server for drone swarm communication. This architecture bypasses the "changing IP" problem by creating a static, private network independent of external routers.

### 1.2 Network Topology

-   **Master (Drone 2):** Acts as the Wi-Fi Access Point (Hotspot). It broadcasts the SSID and manages IP assignment.
    
-   **Slave (Drone 1):** Acts as the Client. It connects to the Master automatically.
    
-   **PC Monitor (Laptop):** Connects to the Master's hotspot to monitor data and SSH into both nodes.
    

### 1.3 Key Requirements

-   No external router required in the field.
    
-   Targeted Point-to-Point (P2P) data delivery.
    
-   Continuous background data transfer.
    

* * *

## 2\. Prerequisites & Core Networking

### 2.1 Reliable Local Access (Ethernet)

Establish a hardwired connection first to avoid locking yourself out:

-   **Option A (DHCP):** Plug the Pi into a router. Ensure Netplan (`/etc/netplan/50-cloud-init.yaml`) has `eth0: dhcp4: true`.
    
-   **Option B (Static):** If direct-connecting to a laptop, set a static IP (e.g., `192.168.1.100/24`) and match the subnet on your laptop.
    

### 2.2 The Ubuntu 24.04 "Networking Quirk"

Ubuntu 24.04 Server uses `networkd` by default. For stable hotspots, we must hand control to **NetworkManager**:

Bash

    sudo apt update
    sudo apt install network-manager

### 2.3 Editing Netplan (The Critical Step)

Open `sudo nano /etc/netplan/50-cloud-init.yaml`. You must add the NetworkManager renderer and disable the `wifis:` section to avoid driver conflicts.

YAML

    network:
      version: 2
      renderer: NetworkManager  # <--- CRITICAL
      ethernets:
        eth0:
          dhcp4: true
          optional: true
      # wifis:                  # <--- Comment out or delete everything below here

**Apply changes:** `sudo netplan apply` (Warning: Existing Wi-Fi SSH sessions will drop).

* * *

## 3\. Configuring the "Master" Drone (Hotspot)

### 3.1 Creating the Hotspot

Bash

    sudo nmcli device wifi hotspot ssid "MasterDrone" password "safedrone" ifname wlan0

**Verify IP:** `ip addr show wlan0` (Expect `10.42.0.1/24`).

### 3.2 Persistent Auto-Start

**Layer 1: Connection Settings**

Bash

    sudo nmcli connection modify "MasterDrone" connection.autoconnect yes
    sudo nmcli connection modify "MasterDrone" connection.autoconnect-priority 100
    sudo nmcli connection modify "MasterDrone" wifi.cloned-mac-address preserve

**Layer 2: The "Safety Net" Kickstart**

Create `sudo nano /etc/rc.local`:

Bash

    #!/bin/bash
    sleep 10
    nmcli connection up "MasterDrone"
    exit 0

`sudo chmod +x /etc/rc.local`

* * *

## 4\. Configuring the "Slave" Drone (Client)

 ### CAUTION
 
 **The "Sawing Off the Branch" Trap:** Do not command the Slave to switch Wi-Fi networks while SSH'd over Wi-Fi. Always use Ethernet for this step.

### 4.1 Manual Field Connection

1.  **Rescan:** `sudo nmcli device wifi rescan`
    
2.  **List:** `sudo nmcli dev wifi list` (Verify "MasterDrone" is visible)
    
3.  **Connect:**
```     
sudo nmcli device wifi connect "MasterDrone" password "safedrone"
```

### 4.2 Troubleshooting "Scanning Not Allowed"

If the radio is "soft-blocked" or sleeping:    
```bash
sudo rfkill unblock wifi
sudo ip link set wlan0 up
```

## 5\. Locating Devices & Access

### 5.1 Finding Slave IP from Master

Run `ip neigh` on the Master. Look for MAC addresses starting with `d8:3a:dd` or `b8:27:eb`.

### 5.2 The JumpHost Method

To SSH into the Slave via the Master from your laptop:

Bash

    ssh -J ubuntu@10.42.0.1 drone1@10.42.0.235

* * *

## 6\. Troubleshooting Log

| **Error** | **Cause** | **Fix** |
| --- | --- | --- |
| **Permission Denied (SSH)** | Password auth disabled | Set `PasswordAuthentication yes` in `/etc/ssh/sshd_config` |
| **Scanning not allowed** | `networkd` conflict | Ensure `renderer: NetworkManager` is set in Netplan |
| **Empty Scan List** | Radio sleeping | `sudo rfkill unblock wifi` |
| **Wi-Fi Crash** | Bluetooth Conflict | Avoid using Bluetooth serial (`rfcomm`) while Wi-Fi is active |

* * *

## 7\. Data Transfer Verification (Netcat)

Test the pipe before writing complex Python/C++ code:

-   **Receiver (Drone 1):** `nc -l -p 12345`
    
-   **Sender (Drone 2):** `echo "Hello Swarm!" | nc 10.42.0.235 12345`
    

* * *

## 8\. Architectural Best Practices

-   **USB Tethering for Internet:** Never use the Wi-Fi chip for both the Hotspot and external Internet. Use a phone via USB for updates/external data.
    
-   **P2P Logic:** Use a bidirectional TCP socket model where each drone runs both a **Listener** (Server) and a **Sender** (Client) script.