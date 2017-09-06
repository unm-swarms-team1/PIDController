function current_setpoint = calcCurrentSetpoint( t, setpoints )
   
    for i = 1:size(setpoints,1)-1
        if (setpoints(i,1) <= t) && (setpoints(i+1,1) >= t)
            current_setpoint = setpoints(i,2);
            return;
        end
    end
    
    if t >= setpoints(end,1)
        current_setpoint = setpoints(end,2);
        return;
    end
    
    disp('Error no setpoint found');
    t
    setpoints

end

