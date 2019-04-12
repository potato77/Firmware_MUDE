# uORB topic for ude attitude control

float32[3] attitude_now
float32[3] attitude_sp

float32[3] atiitude_rate_now

float32[3] integral_ude


float32[3] u_l                      # feedback linearization

float32[3] u_d                      # uncertainties compensation

float32[3] u_total                  # control input (torque)



