
#Imports:
import numpy as np
import random
import cmath
import math
import cv2
from scipy.ndimage import map_coordinates

from matplotlib import image
class ThermalMap:

    def __init__(self, seed = None, length = 100,
                 width = 20, FOV = [55,35],
                 resolution = [32 , 24],
                 pixel_density = 100, time = 13,
                 depth_mine = 0.05,
                 sensor_accuracy= 0.2,  # NEW: The +/- range (e.g., +/- 1.0 C)
                 sensor_bit_depth=None, # NEW: Optional, e.g., 0.1 means steps of 0.1C
                 error_fixed_pattern_amp = 1.0
                 ):

        self.conditions = [0,1,2]

        self.width  = width
        self.length = length
        self.time   = time
        self.FOV    = FOV
        self.res    = resolution

        self.pixel_density     = pixel_density
        self.depth_mine        = depth_mine
        self.static_noise_seed = random.randint(1, 10000)

        #-----------------------------------------------------------------------------------------------------------------
        if seed is not None:
          random.seed(seed)
          np.random.seed(seed)

        self.pixel_len   = pixel_density*self.length
        self.pixel_width = pixel_density*self.width

        # INTERPRETATION OF ACCURACY:
        # If a sensor says "+/- 1C", it usually means 95% of readings are within 1C.
        # In a Normal Distribution, 95% is within 2 standard deviations (2*sigma).
        # So, sigma (std_dev) = accuracy / 2
        self.noise_sigma = sensor_accuracy / 2.0

        # If you want to simulate digital steps (e.g., 0.1C resolution)
        self.quantization_step = sensor_bit_depth

        # Calculating Thermal Values
        #-----------------------------------------------------------------------------------------------------------------
        self._init_Physics();
        #-----------------------------------------------------------------------------------------------------------------

        #Error Constants
        #-----------------------------------------------------------------------------------------------------------------
        # Error Constants for Ahmedabad Drone Survey
        self.error_gps_sigma         = 1.5   # Meters error (x,y) : with normal drone should be 1.5
        self.error_alt_sigma         = 1.5   # Meters error (h)   : with normal drone should be 1.5

        self.error_attn_coeff        = 0.002 # Per meter (Moderate winter air)
        self.error_path_radiance     = 20.0  # Degrees Celsius (Air temp)

        self.error_blur_kernel_len   = 3     # Pixels (Smearing)
        self.error_distortion_k      = -0.1  # Lens factor

        # Electronic Sensor Specs
        self.sensor_accuracy         = 0.2   # +/- 0.2C (Sigma = 0.1, implies NETD ~50mK)
        self.sensor_bit_depth        = 0.1   # High precision digital readout
        self.error_fixed_pattern_amp = 1.0   # Static noise magnitude
        #-----------------------------------------------------------------------------------------------------------------
        #Lens Distortion Condition
        #-----------------------------------------------------------------------------------------------------------------

        fov_horz = self.FOV[0]

        if fov_horz < 65:
          #Very little distortion
          self.error_distortion_k = 0.0
        elif fov_horz > 100:
          #Barrel Distortion : 0.004 per degree over 90
          self.error_distortion_k = 0.15 + (fov_horz - 100) * 0.005
        else:
          self.error_distortion_k = 0.05
        #-----------------------------------------------------------------------------------------------------------------

        #Dead Pixel Map (Broken Silicon)
        # We create a boolean mask: True = Dead, False = Good
        #-----------------------------------------------------------------------------------------------------------------
        prob_dead = 0.004
        raw_prob = np.random.random((self.res[1], self.res[0]))

        self.dead_hot_mask = raw_prob < (prob_dead / 2)          # Stuck at Max
        self.dead_cold_mask = raw_prob > (1 - prob_dead / 2)     # Stuck at Min
        #-----------------------------------------------------------------------------------------------------------------

        #FPN Noise
        #-----------------------------------------------------------------------------------------------------------------
        self.fpn_map = np.random.normal(0, self.error_fixed_pattern_amp, (self.res[1], self.res[0]))
        #-----------------------------------------------------------------------------------------------------------------

        self.grid = np.full([self.pixel_len, self.pixel_width],self.T_base)
        #-----------------------------------------------------------------------------------------------------------------

    def _init_Physics(self):
        #----------------------------------------------------------------------------------------------------------------
        T_mean         = 28                              #T_mean is the long-term average temperature over many days/cycles,
                                                         #Here T_mean is assumed to be average air temperature over the year.

        self.omega          = (2*math.pi)/(86400)            #Sun's angular freq
        self.Q0             = 180                            #Solar Flux amps
        self.t_sec          = (self.time - 12)*3600          #Seconds from solar noon
        self.phase_base     = -1*math.pi/4
        #-----------------------------------------------------------------------------------------------------------------

        # No anomaly Condition Should run for all of them
        #-----------------------------------------------------------------------------------------------------------------
        k_s          = 1.0                                   #Soil Conductivity
        pcp_s        = 1.6e6                                 #Soil Vol. Heat Capacity
        alpha_s      = k_s/(pcp_s)                           #Soil Diffusivity
        self.Z_s     = 1/cmath.sqrt(1j*self.omega*k_s*pcp_s) #Soil Impedence
        self.delta   = math.sqrt(2*alpha_s/self.omega)       #Skin Depth
        self.gamma   = (1 + 1j)/self.delta                   #propogation Constant
        self.A       = self.Q0*abs(self.Z_s)

        T_soil_surface = T_mean + self.A*math.cos(self.omega*self.t_sec + self.phase_base)
        self.T_base    = T_soil_surface
        #-----------------------------------------------------------------------------------------------------------------

    def Thermal_Calculations(self, condition = 0, depth_z = 0.01, thickness_mine = 0.25):
        # condition parameter decides which anomaly is present
        # 0 -> No anomaly
        # 1 -> Mines/Plastic : We will assume case for PVC
        # 2 -> Buried Rocks

        #-----------------------------------------------------------------------------------------------------------------
        if (condition not in self.conditions): raise ValueError("Condition value should be from [0,1,2]")
        Surface_temperature = {}

        omega          = self.omega              #Sun's angular freq
        Q0             = self.Q0                 #Solar Flux amps
        t_sec          = self.t_sec              #Seconds from solar noon
        phase_base     = self.phase_base

        # No anomaly Condition Should run for all of them

        Z_s   = self.Z_s
        delta = self.delta
        gamma = self.gamma
        A     = self.A
        #-----------------------------------------------------------------------------------------------------------------

        if condition == 1:
            # Presence of Mines
            #-----------------------------------------------------------------------------------------------------------------
            depth = depth_z
            k_m   = 0.28
            pcp_m = 1.8e6

            Z_m           = 1/cmath.sqrt(1j*omega*k_m*pcp_m)
            alpha_m       = k_m/(pcp_m)
            delta_m       = math.sqrt(2*alpha_m/omega)
            gamma_m       = (1 + 1j)/delta_m
            tanh_Val      = cmath.tanh(gamma_m * thickness_mine)

            Z_in          = Z_m * (Z_s + Z_m * tanh_Val)/(Z_m + Z_s * tanh_Val)

            gamma_complex = (Z_m - Z_s)/(Z_m + Z_s)
            atten         = cmath.exp(-2*depth/delta)
            phase_anomaly = -2*depth/delta

            complex_factor = gamma_complex * cmath.exp(1j*phase_anomaly)

            delta_T_anom = A*abs(complex_factor)*atten*math.cos(omega*t_sec + cmath.phase(complex_factor) + phase_base)
            # Realism Correction:
            # Small objects lose heat sideways.
            # A 0.7 factor roughly approximates this 3D loss for standard anti-personnel mines.

            shape_factor = 0.7
            self.T_anomaly_delta_peak = float(delta_T_anom.real * shape_factor)
            return float(delta_T_anom.real * shape_factor)
            #-----------------------------------------------------------------------------------------------------------------

        elif condition == 2:

            # Rock Properties (Granite/Basalt)
            k_rock   = 2.8   # High Conductivity
            pcp_rock = 2.2e6 # High Capacity

            # -----------------------------------------------------------------
            # CASE A: Surface Rocks (0 to 2cm deep)
            # -----------------------------------------------------------------
            if depth_z <= 0.02:
                # (Same surface logic as before: Solar heating dominates)
                # ... [Keep previous Surface Rock code] ...
                contrast_mult = 2.0
                delta_T_rocks = A * contrast_mult * math.cos(omega*t_sec + phase_base)
                T_soil_now = A * math.cos(omega*t_sec + phase_base)
                return float(delta_T_rocks.real - T_soil_now) + random.uniform(-1.0, 1.0)

            # -----------------------------------------------------------------
            # CASE B: Buried Rocks (Finite Slab Physics)
            # -----------------------------------------------------------------
            else:
                # 1. Calculate Intrinsic Rock Properties
                Z_rock_mat = 1 / cmath.sqrt(1j * omega * k_rock * pcp_rock)
                alpha_rock = k_rock / pcp_rock
                delta_rock = math.sqrt(2 * alpha_rock / omega) # Skin depth in rock
                gamma_rock = (1 + 1j) / delta_rock

                # 2. Apply Input Impedance Formula (The "Thickness" Fix)
                # We calculate what the thermal wave "sees" when it hits the top of the rock
                tanh_val = cmath.tanh(gamma_rock * thickness_mine)

                numerator   = Z_s + Z_rock_mat * tanh_val
                denominator = Z_rock_mat + Z_s * tanh_val

                Z_effective = Z_rock_mat * (numerator / denominator)

                # 3. Reflection Coefficient
                gamma_complex = (Z_effective - Z_s) / (Z_effective + Z_s)

                # 4. Attenuation & Phase Shift (Travel through soil cover)
                atten         = cmath.exp(-2 * depth_z / delta)
                phase_depth   = -2 * depth_z / delta

                # 5. Final Calculation
                complex_factor = gamma_complex * cmath.exp(1j * phase_depth)
                delta_T_rocks  = A * abs(complex_factor) * math.cos(omega * t_sec + cmath.phase(complex_factor) + phase_base)

                # Shape Factor: Rocks are round, not flat plates.
                # They shed heat sideways more than a plate.
                shape_factor = 0.65

                return float(delta_T_rocks.real * shape_factor)
          #-----------------------------------------------------------------------------------------------------------------

        return 0


    def sigmoidal_function(self, std = 1, xd = 1 ,r=0):
        # xd is decay rate how fast it goes from 0 to 1
        # std is size of flatness distance between both solutions for y=0.5
        #-----------------------------------------------------------------------------------------------------------------
        a = 4*3.4534/xd
        b = a*std/2
        return 1/(1 + np.exp(a*r - b)) - 1/(1 + np.exp(a*r + b))
        #-----------------------------------------------------------------------------------------------------------------

    def add_object(self, x_meter, y_meter, spread_x = 1, spread_y = 1, rotation_deg = 30, radius_px=30, is_mine = 0, depth_z = 0.05, thickness = 0.05):

        center_x_pixel = int(x_meter*self.pixel_density)
        center_y_pixel = int(y_meter*self.pixel_density)
        radius_px      = int(radius_px)

        # Creation of patch
        #-----------------------------------------------------------------------------------------------------------------
        margin = int(self.pixel_density * 0.5) # 0.5 meter margin, for example
        patch_size = radius_px + margin
        x,y = np.ogrid[-patch_size:patch_size, -patch_size:patch_size]
        #-----------------------------------------------------------------------------------------------------------------

        # Rotation of the object
        #-----------------------------------------------------------------------------------------------------------------
        theta = np.radians(rotation_deg)
        x_rot = x*np.cos(theta) - y*np.sin(theta)
        y_rot = x*np.sin(theta) + y*np.cos(theta)
        #-----------------------------------------------------------------------------------------------------------------

        # Calculate Bounds (Clipping safe)
        #-----------------------------------------------------------------------------------------------------------------
        start_x = max(0, center_x_pixel - patch_size)
        start_y = max(0, center_y_pixel - patch_size)
        end_x = min(self.pixel_width, center_x_pixel + patch_size)
        end_y = min(self.pixel_len, center_y_pixel + patch_size)
        #-----------------------------------------------------------------------------------------------------------------

        # Calculate Stamp Offsets
        #-----------------------------------------------------------------------------------------------------------------
        stamp_start_x = 0 if start_x == center_x_pixel - patch_size else (patch_size - (center_x_pixel - start_x))
        stamp_start_y = 0 if start_y == center_y_pixel - patch_size else (patch_size - (center_y_pixel - start_y))
        stamp_end_x = stamp_start_x + (end_x - start_x)
        stamp_end_y = stamp_start_y + (end_y - start_y)
        #-----------------------------------------------------------------------------------------------------------------


        #Heat patch Guassian and Sigmoidal
        #-----------------------------------------------------------------------------------------------------------------
        if is_mine:
            r_px   = np.sqrt(x**2 + y**2)
            #std - xd = radius
            xd_px = self.pixel_density * 0.2  # Moderate fade
            std_px = 2 * radius_px

            delta_temp_mine = self.Thermal_Calculations(condition = 1, thickness_mine= thickness)
            heat_patch = delta_temp_mine * self.sigmoidal_function(std = std_px, xd = xd_px, r = r_px)

            self.grid[start_y:end_y, start_x:end_x] += heat_patch[stamp_start_y:stamp_end_y, stamp_start_x:stamp_end_x]

        else:

            delta_temp = self.Thermal_Calculations(condition = 2, depth_z = depth_z, thickness_mine=thickness)
            heat_patch = delta_temp*np.exp(-1*(x_rot**2/spread_x + y_rot**2/spread_y))

            self.grid[start_y:end_y, start_x:end_x] += heat_patch[stamp_start_y:stamp_end_y, stamp_start_x:stamp_end_x]
        #-----------------------------------------------------------------------------------------------------------------


    def add_mine(self, x_meter_mine, y_meter_mine, mine_radius, thickness = 0.02):

        print(f"Adding Mine at ({x_meter_mine},{y_meter_mine}) ...")
        depth  = self.depth_mine
        depth += random.uniform(-0.2,0.2)

        self.add_object(x_meter= x_meter_mine, y_meter= y_meter_mine,
                        radius_px = mine_radius*self.pixel_density, is_mine = 1, depth_z=depth, thickness = thickness)
        print(f"Mine added sucessfully.")

    def add_anomaly(self, count_surface = 10, count_underground = 20):

      # Lognormal parameters tuned for 1–10 cm rocks
      mu = 1.2          # geometric mean ≈ 3.3 cm (exp(1.2) ≈ 3.32)
      sigma = 0.75      # standard deviation in log-space (controls "spreadiness"

      print(f"Preparing to add {count_surface} surface anomalies...")
      for i in range(count_surface):

          x_meters = random.uniform(0.001*self.width, 0.998*self.width)
          y_meters = random.uniform(0.001*self.length, 0.998*self.length)

          base_rock_px  = np.random.lognormal(mean = mu, sigma = sigma)   #Rocks size from 1cm to 10cm
          spread_rock   = random.uniform(1.0,3.0)                         #Direction spread factor

          if random.choice([True,False]):
              spreadx, spready = base_rock_px*spread_rock, base_rock_px
          else:
              spreadx, spready = base_rock_px, base_rock_px*spread_rock

          depth = 0
          angle = random.uniform(0,360)

          self.add_object(x_meter = x_meters, y_meter = y_meters,spread_x = spreadx,
                          spread_y = spready, depth_z = depth, rotation_deg = angle)

      print(f"Added {count_surface} surface anomalies successfully \n")

      print(f"Preparing to add {count_underground} surface anomalies...")
      for i in range(count_underground):

          x_meters = random.uniform(0.001*self.width, 0.998*self.width)
          y_meters = random.uniform(0.001*self.length, 0.998*self.length)

          base_rock_px  = np.random.lognormal(mean = mu, sigma = sigma)   #Rocks size from 1cm to 10cm
          spread_rock  = random.uniform(1.0,3.0)                          #Direction spread factor

          if random.choice([True,False]):
              spreadx, spready = base_rock_px*spread_rock, base_rock_px
          else:
              spreadx, spready = base_rock_px, base_rock_px*spread_rock

          depth = random.uniform(0.02,0.15)
          angle = random.uniform(0,360)

          # Convert pixel radius to meters
          radius_meters = base_rock_px / self.pixel_density

          # Aspect Ratio: Randomly decide if it's a flat stone (0.2x width) or round (1.0x width)
          aspect_ratio = random.uniform(0.3, 1.2)

          rock_thickness = (radius_meters * 2) * aspect_ratio

          # Clamp limits (e.g., minimum 1cm thickness for physics stability)
          rock_thickness = max(0.01, rock_thickness)

          self.add_object(x_meter = x_meters, y_meter = y_meters,spread_x = spreadx,
                          spread_y = spready, depth_z = depth, rotation_deg = angle, thickness = rock_thickness)
      print(f"Added {count_underground} surface anomalies successfully")

    def add_lens_distortion(self, image_src):
      H, W = image_src.shape
      y, x = np.ogrid[0:H, 0:W]

      #Center the values of grid
      x_centered = x - (W - 1) / 2.0
      y_centered = y - (H - 1) / 2.0

      #Normalization
      scale_factor = max(H, W)/2.0
      x_norm = x_centered/scale_factor
      y_norm = y_centered/scale_factor

      r_sq = x_norm**2 + y_norm**2

      distortion_factor = 1 + self.error_distortion_k * r_sq

      # Map back to source pixels
      x_src = (x_norm * distortion_factor * scale_factor) + (W - 1) / 2.0
      y_src = (y_norm * distortion_factor * scale_factor) + (H - 1) / 2.0

      # Interpolate
      coords = np.array([y_src, x_src])
      image_src = map_coordinates(image_src, coords, order = 1, mode = 'nearest')

      return image_src

    def appy_dead_pixels_mask(self, image_src):
        # dead pixel mask is implemented in constructor such that to not change with each iteration.
        image_src[self.dead_hot_mask] = 100.0
        image_src[self.dead_cold_mask] = -20.0
        return image_src

    def get_view(self, drone_x, drone_y, drone_height):
        #-----------------------------------------------------------------------------------------------------------------
        fov_horz_rad, fov_vert_rad = self.FOV
        fov_horz_rad = math.radians(fov_horz_rad)
        fov_vert_rad = math.radians(fov_vert_rad)
        #-----------------------------------------------------------------------------------------------------------------

        ground_view_hor = 2 * (drone_height + np.random.normal(0.0, self.error_alt_sigma)) * math.tan(fov_horz_rad / 2)
        ground_view_ver = 2 * (drone_height + np.random.normal(0.0, self.error_alt_sigma)) * math.tan(fov_vert_rad / 2)
        #-----------------------------------------------------------------------------------------------------------------

        section_slice_x = int(ground_view_hor * self.pixel_density)
        section_slice_y = int(ground_view_ver * self.pixel_density)
        #-----------------------------------------------------------------------------------------------------------------

        half_slice_x    = section_slice_x // 2
        half_slice_y    = section_slice_y // 2
        #-----------------------------------------------------------------------------------------------------------------

        drone_x_px = int((drone_x + np.random.normal(0.0, self.error_gps_sigma)) * self.pixel_density)   #Addition of drone Jitter error due to percision limitation
        drone_y_px = int((drone_y + np.random.normal(0.0, self.error_gps_sigma)) * self.pixel_density)   #Same error addition as above
        #-----------------------------------------------------------------------------------------------------------------

        #Start and End locations on the main map from drone view
        #-----------------------------------------------------------------------------------------------------------------
        start_x    = max(0, drone_x_px - half_slice_x)
        start_y    = max(0, drone_y_px - half_slice_y)
        end_x      = min(self.pixel_width, drone_x_px + half_slice_x)
        end_y      = min(self.pixel_len, drone_y_px + half_slice_y)

        map_section = self.grid[start_y:end_y,start_x:end_x]

        # For Edge case scenario we will pad and resize the output
        #-----------------------------------------------------------------------------------------------------------------
        if map_section.shape != (section_slice_x, section_slice_y):

            padded = np.full((section_slice_y, section_slice_x), self.T_base)

            #----------------------------------------------------------------------------------
            # The idea here is that when drone moves near edge of the map it should still receive
            # full image input in real life, but here we are going out of the array so all outer
            # border of array needs to give padded output for such cases.
            #----------------------------------------------------------------------------------

            ideal_start_x = drone_x_px - half_slice_x
            ideal_start_y = drone_y_px - half_slice_y

            height, width = map_section.shape
            start_loc_x   = abs(start_x - ideal_start_x)
            start_loc_y   = abs(start_y - ideal_start_y)

            padded[start_loc_y:start_loc_y + height, start_loc_x:start_loc_x + width] = map_section

            map_section = padded

        sensor_res_x, sensor_res_y = self.res
        clear_view = cv2.resize(map_section, (sensor_res_x, sensor_res_y), interpolation=cv2.INTER_AREA)
        clear_view = np.clip(clear_view, -20, 150)
        #-----------------------------------------------------------------------------------------------------------------

        # Atmospheric Attenuation Error Addition
        #-------------------------------------------------------------------------------------
        transmission = math.exp(-self.error_attn_coeff * drone_height)
        map_section  = map_section * transmission + self.error_path_radiance * (1 - transmission)
        #-------------------------------------------------------------------------------------

        #Motion blur Implementation
        #I couldn't

        #Lens Distortion Error Implementation. :r_src​=r_dst​⋅(1+k⋅r_dst^2​)
        #------------------------------------------------------------
        map_section = self.add_lens_distortion(map_section)

        sensor_res_x, sensor_res_y = self.res
        sensor_view = cv2.resize(map_section, (sensor_res_x, sensor_res_y), interpolation=cv2.INTER_AREA)
        #------------------------------------------------------------

        # Fixed Pattern Noise : Pixel_final​=Pixel_measured​+Noise_static​[row][col]
        #------------------------------------------------------------
        sensor_view += self.fpn_map
        #------------------------------------------------------------

        sensor_view = self.appy_dead_pixels_mask(sensor_view)

        # 1. Add Random Noise (NETD)
        # We use the sigma calculated from your "+/- 1C" requirement
        sensor_noise = np.random.normal(0, self.noise_sigma, (sensor_res_y, sensor_res_x))
        sensor_view += sensor_noise

        # If your sensor can only read in 0.5C steps, 28.3C becomes 28.5C
        if self.quantization_step is not None:
            sensor_view = np.round(sensor_view / self.quantization_step) * self.quantization_step

        sensor_view = np.clip(sensor_view, -20, 150)

        return sensor_view, clear_view

    def get_sky_view(self, T_uniform=0.0):

        sensor_res_x, sensor_res_y = self.res
        sensor_view = np.full((sensor_res_y, sensor_res_x), T_uniform, dtype=np.float32)

        sensor_view += self.fpn_map
        sensor_view = self.appy_dead_pixels_mask(sensor_view)
        sensor_noise = np.random.normal(0, self.noise_sigma, (sensor_res_y, sensor_res_x))
        sensor_view += sensor_noise


        if self.quantization_step is not None:
            sensor_view = np.round(sensor_view / self.quantization_step) * self.quantization_step

        sensor_view = np.clip(sensor_view, -50, 150)

        return sensor_view
