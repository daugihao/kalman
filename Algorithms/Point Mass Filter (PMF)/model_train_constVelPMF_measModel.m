function [p] = model_train_constVelPMF_measModel(y,measEst)
    
    %% Set up Gaussian parameters
    % Gaussian amplitude
    A = 1; 
    % Centre of gaussian
    x0 = 0;
    % Standard deviation
    sigmax = 1;
    
    %% Difference between grid point and estimated state
    difference = y-measEst;
    
    Comp1 = (difference(1)-x0)^2 / 2*sigmax^2;
    
    p = A*exp(-(Comp1));

end