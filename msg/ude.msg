# uORB topic for ude attitude control

float32 start_time

float32 input_time

float32 thrust_sp

float32[3] attitude_ref
float32[3] attitude_dot_ref
float32[3] attitude_dot_ref_hpf
float32[3] attitude_ddot_ref
float32[3] attitude_dddot_ref

float32[3] attitude_now
float32[3] attitude_rate_now

float32[3] error_attitude
float32[3] error_attitude_rate

float32[3] feedforward

float32[3] u_l_kp                     # feedback linearization
float32[3] u_l_kd
float32[3] u_l_km              ## only for mUDE
float32[3] u_d

float32[3] torque_ref           ## only for mUDE
float32[3] torque_est           ## only for mUDE

float32[3] f1_est           ## only for mUDE
float32[3] f1_dot_est           ## only for mUDE
float32[3] f2_est           ## only for mUDE
float32[3] f_est           ## only for mUDE
float32[3] f2

float32[3] u_total                  # control input (torque)



