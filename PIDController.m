% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )
   
torque1 = 0;
torque2 = 0; 

control_forces = [torque1; torque2];

end
