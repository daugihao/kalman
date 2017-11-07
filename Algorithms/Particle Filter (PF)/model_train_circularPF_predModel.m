function [p] = model_train_circularPF_predModel(meanEst)
    
    %% Set up Gaussian parameters
    % Standard deviation
    sigmax = 0.01; sigmay = 0.01;
    
    p = zeros(size(meanEst));
    
    p(1) = meanEst(1) + sigmax*randn;
    p(2) = meanEst(2) + sigmay*randn;

end