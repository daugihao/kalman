function [p] = model_train_circularPF_predModel(x,meanEst)
    
    %% Set up Gaussian parameters
    % Standard deviation
    sigmax = 0.3; sigmay = 0.3;
    
    %% Difference between grid point and estimated state
    difference = x-meanEst;
    
    Comp1 = difference(1)^2 / (2*sigmax^2);
    Comp2 = difference(2)^2 / (2*sigmay^2);
    
    p = exp(-(Comp1 + Comp2));

end