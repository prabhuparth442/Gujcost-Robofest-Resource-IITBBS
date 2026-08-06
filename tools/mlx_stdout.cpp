#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include "../../lib/mlx90640-library/headers/MLX90640_API.h"
#include "../../lib/mlx90640-library/headers/MLX90640_I2C_Driver.h"

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
