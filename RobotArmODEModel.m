function xdot=RobotArmODEModel(t,x,joint1_setpoints, joint2_setpoints, spec, Kpid)
xdot=zeros(8,1);

% Current setpoint
joint1_angle_desired = calcCurrentSetpoint(t, joint1_setpoints);
joint2_angle_desired = calcCurrentSetpoint(t, joint2_setpoints);

%% Robot Specifications
M1=spec(3); % mass (kg)
M2=spec(4); % mass (kg)
L1=spec(1); % length (m)
L2=spec(2); % length (m)
g=9.8; % Newtons
joint_actuator_force_limit = spec(5); % N m (Newton meters)

% Read state
joint1_angle = x(3);
joint2_angle = x(4);
link1_velocity = x(5);
link2_velocity = x(6);

%% Inertial Forces
inertial_forces = calcInertialForces(joint2_angle, M1, M2, L1, L2);

%% Coriolis Forces
coriolis_forces = calcCoriolisForces(joint2_angle, link1_velocity, link2_velocity, M2, L1, L2 );

%% Gravity Forces
gravitational_forces = calcGravitationalForces(g, joint1_angle, joint2_angle, M1, M2, L1, L2);

%% PID Control Forces   
control_forces = PIDController(t, joint1_angle_desired, joint2_angle_desired, joint1_angle, joint2_angle, link1_velocity, link2_velocity);

% Limit actuation forces
if abs(control_forces(1)) > joint_actuator_force_limit
    control_forces(1) = joint_actuator_force_limit*sign(control_forces(1));
end

if abs(control_forces(2)) > joint_actuator_force_limit
    control_forces(2) = joint_actuator_force_limit*sign(control_forces(2));
end

% Calculate the second derivative of the system (i.e. the new accelerations or forces)
q_ddot=inertial_forces\(-coriolis_forces-gravitational_forces)+inertial_forces*control_forces;

%% System states for the next step of the ODE solver
xdot(1)=(joint1_angle_desired-joint1_angle); % record error this step 
xdot(2)=(joint2_angle_desired-joint2_angle); % record error this step
xdot(3)=x(5); %theta1-dot
xdot(4)=x(6); %theta2-dot
xdot(5)=q_ddot(1); %theta1-2dot
xdot(6)=q_ddot(2); %theta2-2dot

% Record the control forces for later plotting
xdot(7)=control_forces(1);
xdot(8)=control_forces(2);
end