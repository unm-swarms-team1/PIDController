% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

k_p = 0.0019;
k_d = 0.00002;
k_i = 0.0000048;

persistent error_integral
persistent last_time

error = [joint1_angle_setpoint - joint1_measured_angle; joint2_angle_setpoint - joint2_measured_angle];

if isempty(last_time) 
    last_time = 0.0;
end
dt = current_time - last_time;
last_time = current_time;       

if isempty(error_integral)
    error_integral = error*dt;
else
    error_integral = error_integral + (error*dt);
end

error_dot = [error1_dot; error2_dot];

torque = k_p * error + k_d * error_dot + k_i * error_integral;

control_forces = torque;

end
