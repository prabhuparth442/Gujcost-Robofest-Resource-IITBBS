/*
 * mlx_stdout.cpp  —  MLX90640 Thermal Camera Frame Streamer
 * ==========================================================
 * This C++ binary reads from the MLX90640 32×24 thermal sensor over I²C
 * and streams raw float32 temperature arrays to stdout in a tight loop.
 *
 * Python reads from this process via subprocess pipe:
 *   proc = subprocess.Popen(["./bin/mlx_stdout"], stdout=subprocess.PIPE)
 *   raw  = proc.stdout.read(768 * 4)        # 768 pixels × 4 bytes each
 *   frame = np.frombuffer(raw, dtype=np.float32).reshape((24, 32))
 *
 * WHY C++ instead of Python?
 *   The MLX90640 requires precise I²C timing for EEPROM access and
 *   per-frame DMA.  The Melexis C++ driver handles this reliably.
 *   Python-level I²C (smbus2) introduces timing jitter that causes
 *   frequent frame drops and EEPROM read failures.
 *
 * Output format (stdout):
 *   Infinite stream of 3072-byte chunks (768 × sizeof(float)).
 *   Each chunk = one 24×32 frame, pixels in row-major order.
 *   Temperature values are in degrees Celsius.
 *   Debug messages go to stderr (do NOT mix with stdout data).
 *
 * Build:
 *   Requires the MLX90640 library built from:
 *     drone_swarm_folder/drone_swarm/lib/mlx90640-library/
 *   and BCM2835 library installed system-wide.
 *
 *   g++ -o ../bin/mlx_stdout mlx_stdout.cpp \
 *       -I../../drone_swarm_folder/drone_swarm/lib/mlx90640-library/headers \
 *       -L../../drone_swarm_folder/drone_swarm/lib/mlx90640-library \
 *       -lMLX90640_API -lbcm2835
 *
 * Run:
 *   sudo ./bin/mlx_stdout
 *   (sudo needed for BCM2835 hardware access)
 *
 * Sensor:
 *   MLX90640 at I²C address 0x33.
 *   Refresh rate set to 2 Hz (adequate for 0.5 m step scan at ≤0.3 m/s).
 *   Emissivity = 0.95 (good for soil, plastic discs, wet ground).
 *   Reflected temperature = Ta − 8°C (standard ambient correction).
 */

#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include "../../drone_swarm_folder/drone_swarm/lib/mlx90640-library/headers/MLX90640_API.h"
#include "../../drone_swarm_folder/drone_swarm/lib/mlx90640-library/headers/MLX90640_I2C_Driver.h"

int main() {
    fprintf(stderr, "[C++ DEBUG] Starting MLX90640 Pipeline...\n");

    const uint8_t mlxAddress = 0x33;
    uint16_t eeMLX90640[832];
    float mlx90640To[768];
    uint16_t mlx90640Frame[834];
    paramsMLX90640 mlx90640;

    fprintf(stderr, "[C++ DEBUG] Initializing I2C bus...\n");
    MLX90640_I2CInit();

    fprintf(stderr, "[C++ DEBUG] Setting Refresh Rate to 2Hz...\n");
    MLX90640_SetRefreshRate(mlxAddress, 0x02);

    fprintf(stderr, "[C++ DEBUG] Dumping EEPROM...\n");
    int status = MLX90640_DumpEE(mlxAddress, eeMLX90640);
    if (status != 0) {
        fprintf(stderr, "[C++ FATAL] Failed to read EEPROM. Status: %d. Check I2C permissions (sudo) or wiring.\n", status);
        return -1;
    }

    fprintf(stderr, "[C++ DEBUG] Extracting Parameters...\n");
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    fprintf(stderr, "[C++ DEBUG] Entering Capture Loop...\n");
    while (true) {
        int frameStatus = MLX90640_GetFrameData(mlxAddress, mlx90640Frame);
        if (frameStatus < 0) {
            // Uncomment the line below if you want to see every single dropped frame
            fprintf(stderr, "[C++ WARNING] GetFrameData timeout/noise: %d\n", frameStatus);
            continue;
        }

        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - 8.0f;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95f, tr, mlx90640To);

        // Blast the float array to Python
        fwrite(mlx90640To, sizeof(float), 768, stdout);
        fflush(stdout); 
    }
    return 0;
}
