# Download and Build
1. Install PX4 develop environment (This firmware is based on 1.8.0)
   
   Please ref to Developer Guide: https://dev.px4.io/
   
2. Git clone this repository

   mkdir -p ~/px4_firmware

   cd px4_firmware

   git clone https://github.com/potato77/Firmware

   cd Firmware
   
   Pixhawk 1

   make px4fmu-v3_default 

   Pixhawk 4

   make px4fmu-v5_default 

   jMAVSim Simulation

   make posix jmavsim


# Parameter setting

For manual mode, set EKF2_AID_MASK to 0 (use GPS) and EKF2_HGT_MODE to 0 (Barometric pressure).

For offboard mode, set EKF2_AID_MASK to 24 (vision position fusion and vision yaw fusion) and EKF2_HGT_MODE to 3 (Vision).

# UDE Attitude Controller

UDE_SWITCH = 0 : Default cascade PID attitude controller

UDE_SWITCH = 1 : cascade UDE attitude controller

UDE_SWITCH = 2 : PD+UDE attitude controller
