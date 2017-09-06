function Gq = calcGravitationalForces( g, joint1_angle, joint2_angle, M1, M2, L1, L2 )

g1=-(M1+M2)*g*L1*sin(joint1_angle)-M2*g*L2*sin(joint1_angle+joint2_angle);
g2=-M2*g*L2*sin(joint1_angle+joint2_angle);
Gq=[g1;g2];

end

