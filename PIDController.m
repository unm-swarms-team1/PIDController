% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

k_p = 0.0019;
k_d = 0.00002;
k_i = 0.0000048;

persistent error_integral

error = [joint1_angle_setpoint - joint1_measured_angle; joint2_angle_setpoint - joint2_measured_angle];

if isempty(error_integral)
    error_integral = error;
else
    error_integral = error_integral + error;
end

torque = k_p * error + k_d * [error1_dot; error2_dot] + k_i * error_integral;

control_forces = torque;

end
