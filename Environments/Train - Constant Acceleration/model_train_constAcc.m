function [d] = model_train_constAcc(s,dt)
% MODEL_TRAIN_CONSTACC Model setup

d.X = zeros(s.NState,length(s.t));

% Previous state (initial guess)
d.X(:,1) = [0; 
    0.5;
    0];

% Motion equation: X = F*X_prev + Noise, that is X(n) = X(n-1) + V(n-1) * dt
% Of course, V is not measured, but it is estimated
% F represents the dynamics of the system: it is the motion equation
d.F = [1 dt 0;
       0  1 dt
       0 0 1];

% The error matrix (or the confidence matrix): P states whether we should 
% give more weight to the new measurement or to the model estimate 
d.sigma_model = 1;
d.P = [d.sigma_model^2             0 0;
                 0 d.sigma_model^2   0
                 0 0 d.sigma_model^2];
d.P1 = zeros(size(d.P));


% Q is the process noise covariance. It represents the amount of
% uncertainty in the model. In our case, we arbitrarily assume that the model is perfect (no
% acceleration allowed for the train, or in other words - any acceleration is considered to be a noise)
d.Q = [0 0 0;
     0 0 0;
     0 0 0];

% H is the measurement matrix. 
% We measure X, so H(1) = 1
% We do not measure V, so H(2)= 0
d.H = [1 0 0];

% R is the measurement noise covariance. Generally R and sigma_meas can
% vary between samples. 
d.sigma_meas = s.sigma_meas;
d.R = s.sigma_meas^2;