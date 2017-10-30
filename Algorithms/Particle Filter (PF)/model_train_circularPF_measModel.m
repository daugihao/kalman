function [p] = model_train_circularPF_measModel(y,measEst)
    
    %% Set up Gaussian parameters
    % Standard deviation
    sigmax = 0.01;
    
    %% Difference between grid point and estimated state
    difference = y-measEst;
    
    Comp1 = difference(1)^2 / (2*sigmax^2);
    
    p = exp(-(Comp1));

end