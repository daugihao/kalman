function [p] = model_train_constVelPMF_predModel(x,meanEst)
    
    %% Set up Gaussian parameters
    % Gaussian amplitude
    A = 1; 
    % Centre of gaussian
    x0 = 0; y0 = 0; 
    % Standard deviation
    sigmax = 0.1; sigmay = 0.1;
    
    %% Difference between grid point and estimated state
    difference = x-meanEst;
    
    Comp1 = (difference(1)-x0)^2 / 2*sigmax^2;
    Comp2 = (difference(2)-y0)^2 / 2*sigmay^2;
    
    p = A*exp(-(Comp1 + Comp2));

end