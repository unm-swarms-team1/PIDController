function Cq = calcCoriolisForces( joint2_angle, link1_acceleration, link2_acceleration, M2, L1, L2 )

c1=-M2*L1*L2*sin(joint2_angle)*(2*link1_acceleration*link2_acceleration+link2_acceleration^2); 
c2=-M2*L1*L2*sin(joint2_angle)*link2_acceleration*link2_acceleration; 
Cq=[c1;c2];

end

