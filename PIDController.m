% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

k_p = 4000;
k_d = 200;
k_i = 4000;

persistent error_integral
persistent last_time
persistent last_error
persistent error_dot

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

if isempty(last_error)
    last_error = [0.0;0.0];
end

if isempty(error_dot)
    error_dot = [0.0;0.0];
end

if  dt ~= 0.0
    error_dot = (error - last_error)./dt;
end

last_error = error;

torque = k_p * error + k_d * error_dot + k_i * error_integral;

control_forces = torque;

end
