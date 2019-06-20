# uORB topic for ude attitude control

float32 dt

float32 thrust_sp

float32[3] attitude_now
float32[3] attitude_sp
float32[3] error_attitude

float32[3] attitude_sp_dot
float32[3] attitude_sp_dot_fhan
float32[3] attitude_sp_dot_hpf
float32[3] attitude_rate_now
float32[3] error_attitude_rate

float32[3] u_l_kp                     # feedback linearization
float32[3] u_l_kd

float32[3] u_d_ev                      # uncertainties compensation
float32[3] u_d_int
float32[3] u_d

float32[3] u_total                  # control input (torque)



