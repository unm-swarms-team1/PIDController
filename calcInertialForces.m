function Bq = calcInertialForces( angle, M1, M2, L1, L2 )

b11=(M1+M2)*L1^2+M2*L2^2+2*M2*L1*L2*cos(angle); 
b12=M2*L2^2+M2*L1*L2*cos(angle); 
b21=M2*L2^2+M2*L1*L2*cos(angle);
b22=M2*L2^2;
Bq=[b11 b12;b21 b22];

end

