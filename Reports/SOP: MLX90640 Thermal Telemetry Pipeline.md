# SOP: MLX90640 Thermal Telemetry Pipeline

## 1. Hardware Integration

The 7semi MLX90640 uses the I2C protocol. It requires 4 physical connections to the Raspberry Pi GPIO header.

* **VIN (VCC):** Pin 1 (3.3V Power)
* **GND:** Pin 6 (Ground)
* **SDA (Data):** Pin 3 (GPIO 2)
* **SCL (Clock):** Pin 5 (GPIO 3)

*(Insert your physical wiring photos here)*

---

## 2. Kernel Configuration (I2C Overclocking)

By default, the Raspberry Pi I2C bus runs at 100kHz, which will choke the thermal sensor and cause "Too many retries" errors. You must alter the boot firmware to expand the bandwidth.

Open the boot configuration:
`sudo nano /boot/firmware/config.txt`

**For Python (Max 8Hz):**
Ensure this exact line is present to set the bus to 400kHz.
`dtparam=i2c_arm=on,i2c_arm_baudrate=400000`

**For C++ (Max 32Hz - 64Hz):**
Ensure this exact line is present to set the bus to 1MHz.
`dtparam=i2c_arm=on,i2c_arm_baudrate=1000000`

**Critical Step:** You must run `sudo reboot` for the hardware clock changes to take effect.

---

## 3. The Python Pipeline (Prototyping & Low-Speed)

Python is used for validation and prototyping up to 8Hz.

### A. Library Installation

Do not attempt to install the outdated `PIL` package. You must operate inside a virtual environment to bypass Ubuntu's package restrictions.

```bash
source thermal_env/bin/activate
pip install adafruit-circuitpython-mlx90640 numpy pillow

```

### B. Python Rolling Queue Script (`python_queue.py`)

This script uses a Ring Buffer. It captures data continuously, retaining only the newest 1,000 frames in RAM. When you stop the script, it dumps the queue to a `.npy` file. We attach the timestamp to the 0th index of every frame.

```python
import time
import board
import busio
import adafruit_mlx90640
import numpy as np
from collections import deque

# Hardware Setup
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ

# Queue Setup: deque automatically deletes the oldest item when it hits maxlen
MAX_FRAMES = 1000
ring_buffer = deque(maxlen=MAX_FRAMES)
raw_frame = [0] * 768

print(f"Rolling buffer active. Collecting up to {MAX_FRAMES} frames. Press Ctrl+C to save and exit.")

try:
    while True:
        try:
            mlx.getFrame(raw_frame)
            current_time = time.time()
            
            # Combine timestamp and frame into a single 769-element array
            frame_with_time = np.concatenate(([current_time], raw_frame))
            ring_buffer.append(frame_with_time)
            
            print(f"Queue Size: {len(ring_buffer)}/{MAX_FRAMES}", end='\r')
            time.sleep(0.1)
            
        except ValueError:
            continue # Catch I2C bus collisions and retry
            
except KeyboardInterrupt:
    print("\nCapture stopped. Dumping RAM to SD card...")
    # Convert deque to cohesive 3D stack and save
    dataset = np.array(ring_buffer)
    np.save("python_rolling_queue.npy", dataset)
    print("Saved as 'python_rolling_queue.npy'.")

```

**Function Explanations:**

* `collections.deque(maxlen=1000)`: A highly optimized queue. If it holds 1000 items and you add 1 more, it instantly drops item #1 to make room for item #1001. Perfect for rolling memory.
* `np.concatenate()`: Glues the timestamp float and the 768 temperature floats into one contiguous array for easy saving.

---

## 4. The C++ Pipeline (High-Speed Drone Deployment)

**Why C++?** Python is an interpreted language; it recalculates its own logic on every loop. At 32Hz, Python cannot process the I2C byte math fast enough, causing frame drops and bus lockups. C++ is compiled directly into machine code, allowing zero-latency processing required for high-speed drone telemetry.

### A. Library Installation

You must use the Pimoroni Linux fork of the Melexis drivers and strictly include standard integer definitions.

```bash
sudo apt install libi2c-dev build-essential git -y
git clone https://github.com/pimoroni/mlx90640-library.git
cd mlx90640-library

```

### B. C++ Rolling Queue Script (`cpp_queue.cpp`)

This script implements a native circular array in C++. It overwrites the oldest data continuously. On exit, it reorders the data chronologically and writes a native `.npy` file.

```cpp
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <string>
#include <chrono>
#include <csignal>
#include "headers/MLX90640_API.h"
#include "headers/MLX90640_I2C_Driver.h"

const int MAX_FRAMES = 1000;
float ringBuffer[MAX_FRAMES][769]; // Index 0 is Timestamp, 1-768 is Data
int head = 0;
bool bufferFull = false;
volatile sig_atomic_t stop = 0;

// Catch Ctrl+C to safely trigger the file save
void handle_sigint(int sig) { stop = 1; }

void write_npy_header(std::ofstream& out, int frames) {
    std::string dict = "{'descr': '<f4', 'fortran_order': False, 'shape': (" +
                       std::to_string(frames) + ", 769), }";
    int remainder = 64 - (10 + dict.length()) % 64;
    dict.append(remainder, ' ');
    dict.back() = '\n'; 
    const char magic[] = "\x93NUMPY\x01\x00";
    out.write(magic, 8);
    uint16_t dict_len = dict.length();
    out.write(reinterpret_cast<const char*>(&dict_len), 2);
    out.write(dict.c_str(), dict.length());
}

int main() {
    std::signal(SIGINT, handle_sigint);

    const uint8_t mlxAddress = 0x33;
    uint16_t eeMLX90640[832];
    uint16_t mlx90640Frame[834];
    paramsMLX90640 mlx90640;

    MLX90640_I2CInit();
    MLX90640_SetRefreshRate(mlxAddress, 0x05); // 32Hz
    MLX90640_DumpEE(mlxAddress, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    std::cout << "32Hz Rolling Buffer Active. Press Ctrl+C to save." << std::endl;

    while (!stop) {
        if (MLX90640_GetFrameData(mlxAddress, mlx90640Frame) < 0) continue; 

        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - 8.0f;
        
        // Get precise timestamp
        auto now = std::chrono::system_clock::now().time_since_epoch();
        float current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now).count() / 1000.0f;

        // Store Timestamp at index 0
        ringBuffer[head][0] = current_time;
        // Store 768 floats starting at index 1
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95f, tr, &ringBuffer[head][1]);

        head++;
        if (head >= MAX_FRAMES) {
            head = 0;
            bufferFull = true;
        }
    }

    // --- SAVE LOGIC ---
    std::cout << "\nSaving dataset..." << std::endl;
    int total_frames = bufferFull ? MAX_FRAMES : head;
    std::ofstream outFile("cpp_rolling_queue.npy", std::ios::binary);
    write_npy_header(outFile, total_frames);

    if (bufferFull) {
        // Write the oldest data first (from head to end of array)
        outFile.write(reinterpret_cast<const char*>(&ringBuffer[head][0]), (MAX_FRAMES - head) * 769 * sizeof(float));
    }
    // Write the newest data (from 0 to head)
    outFile.write(reinterpret_cast<const char*>(&ringBuffer[0][0]), head * 769 * sizeof(float));

    outFile.close();
    std::cout << "Data committed to 'cpp_rolling_queue.npy'." << std::endl;
    return 0;
}

```

**Function Explanations:**

* `std::signal(SIGINT, handle_sigint)`: A hardware interrupt watcher. It intercepts the `Ctrl+C` command so the program doesn't just crash; it smoothly stops the capture loop and moves to the save logic.
* `auto now = std::chrono::system_clock::now().time_since_epoch()`: The C++ standard way to pull the exact millisecond timestamp from the operating system kernel.
* `volatile sig_atomic_t stop`: A special variable type that is immune to caching. It ensures the while loop immediately sees when `Ctrl+C` changes its value to 1.
* *Circular Math:* By using a `head` counter that resets to 0 when it hits 1000, we endlessly loop over the same memory block without ever leaking RAM or crashing the Pi.

### C. Compilation Command

`g++ -O3 -include stdint.h cpp_queue.cpp functions/MLX90640_API.cpp functions/MLX90640_LINUX_I2C_Driver.cpp -I headers -o cpp_queue`

