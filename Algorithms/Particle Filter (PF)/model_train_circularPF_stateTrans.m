function [y] = model_train_circularPF_stateTrans(x,dt)
       
    y = zeros(size(x));
    
    theta = atan2(x(2),x(1));
    omegat = 10*dt;
    thetanew = theta + omegat;
    y(1) = cos(thetanew);
    y(2) = sin(thetanew);
    
end