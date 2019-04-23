# uORB topic for ude attitude control

float32 thrust_sp

float32[3] attitude_now
float32[3] attitude_sp

float32[3] attitude_rate_now

float32[3] u_kp                     # feedback linearization
float32[3] u_kd

float32[3] u_f                      # uncertainties compensation

float32[3] u_total                  # control input (torque)



