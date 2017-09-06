
function robotarm2(time_max, link1_len, link2_len, link1_mass, link2_mass, torque_limit, joint1_init_angle, joint2_init_angle, joint1_desired_angles, joint2_desired_angles)
%% Initilization
close all % Close previous figures
th_int=[joint1_init_angle, joint2_init_angle]; %initial positions 
x0=[0 0 th_int 0 0 0 0]; %states initial values
Ts=[0 time_max]; %time span
%% Robot Specifications
L1=link1_len; %link 1
L2=link2_len; %link 2
M1=link1_mass; %mass 1
M2=link2_mass; %mass 2
spec=[L1 L2 M1 M2 torque_limit];

%% ODE solving
%opt1=odeset('RelTol',1e-10,'AbsTol',1e-20,'NormControl','off'); 
[T,X] = ode45(@(t,x) RobotArmODEModel(t,x,joint1_desired_angles,joint2_desired_angles,spec),Ts,x0);
%% Output
joint1_angles=X(:,3); 
joint2_angles=X(:,4);  

% Unpack data we stored in the ODE model state. The state is summed by the ODE45
% function so we have to take derivative to get the oriinal value. This is
% a hack.
F1=diff(X(:,7))./diff(T); 
F2=diff(X(:,8))./diff(T); 
error1 = diff(X(:,1))./diff(T);
error2 = diff(X(:,2))./diff(T);
tt=0:(T(end)/(length(F1)-1)):T(end);

%Convert angles to cartesian coordinates
x1=L1.*sin(joint1_angles); % X1
y1=L1.*cos(joint1_angles); % Y1 
x2=L1.*sin(joint1_angles)+L2.*sin(joint1_angles+joint2_angles); % X2 
y2=L1.*cos(joint1_angles)+L2.*cos(joint1_angles+joint2_angles); % Y2

% Show an animation of the arm
LT = 12.0;
JR = 0.05;
O = [0,0];
bounds_padding = 0.1;
axis(gca,'equal');
animation_bounds = [-max(abs([x1;x2;y1;y2])), max(abs([x1;x2;y1;y2])), -max(abs([x1;x2;y1;y2])), max(abs([x1;x2;y1;y2]))]*(1+bounds_padding);
axis(animation_bounds);

grid on
for i=2:length(T)
    rod1 = line([O(1), x1(i)], [O(2), y1(i)], 'LineWidth', LT);
    rod2 = line([x1(i), x2(i)], [y1(i), y2(i)], 'LineWidth', LT);
    O_circ = viscircles(O, JR,'LineWidth', LT/2);
    ball1 = viscircles([x1(i), y1(i)], JR,'LineWidth', LT/2);
    ball2 = viscircles([x2(i), y2(i)], JR, 'LineWidth', LT/2);
    
    current_time = text(animation_bounds(1)+0.1,animation_bounds(4)-0.1,[num2str((i/length(T))*time_max), ' (s)']);
    pause(0.001);
    
    % Clean up between frames
    if ( i < length(T) )
        delete(rod1);
        delete(rod2);
        delete(ball1);
        delete(ball2);
        delete(O_circ);
        delete(current_time);
    end

end

%theta1 error plot
subplot(2,2,1)
plot(tt,error1)
grid
title('\theta_1 error')
ylabel('\theta_1 error (rad)')
xlabel('time (s)')

%theta2 error plot
subplot(2,2,2)
plot(tt,error2)
grid
title('\theta_2 error')
ylabel('\theta_2 error (rad)')
xlabel('time (s)')

%torque1 plot
subplot(2,2,3)
plot(tt,F1)
grid
title('Torque of \theta_1')
ylabel('\theta_1 torque')
xlabel('time (s)')

%torque2 plot
subplot(2,2,4)
plot(tt,F2)
grid
title('Torque of \theta_2')
ylabel('\theta_2 torque')
xlabel('time (s)')
end