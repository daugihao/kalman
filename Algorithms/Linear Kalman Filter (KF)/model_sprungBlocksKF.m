function [d] = model_sprungBlocksKF(s,dt)
% MODEL_SPRUNGBLOCKS Model setup

d.typeString = 'Linear KF';

d.X = zeros(s.NState,length(s.t));

% Previous state (initial guess)
d.X(:,1) = [0; 
    0;
    1;
    1;
    0;
    -1];

k = 1;
m = 1;

% Motion equation: X = F*X_prev + Noise, that is X(n) = X(n-1) + V(n-1) * dt
% Of course, V is not measured, but it is estimated
% F represents the dynamics of the system: it is the motion equation
d.F = [1 dt 0 0 0 0;
       0  1 dt 0 0 0;
       -k/m 0 0 k/m 0 0;
       0 0 0 1 dt 0;
       0 0 0 0 1 dt
       k/m 0 0 -k/m 0 0];

% The error matrix (or the confidence matrix): P gives the confidence in
% the initial estimate. A low value indicates that the initial state should
% be trusted (it represents the level of error associated with that
% measurement).
d.P = [1e-9 0 0 0 0 0;
       0 1e-9 0 0 0 0;
       0 0 1e-9 0 0 0;
       0 0 0 1e-9 0 0;
       0 0 0 0 1e-9 0;
       0 0 0 0 0 1e-9];
d.P1 = zeros(size(d.P));


% Q is the process noise covariance. It represents the amount of
% uncertainty in the model. In our case, we arbitrarily assume that the model is perfect (no
% acceleration allowed for the train, or in other words - any acceleration
% is considered to be a noise). The elements represent variance magnitude.
d.Q = [0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];

% H is the measurement matrix. 
% We measure X, so H(1) = 1
% We do not measure V, so H(2)= 0
d.H = [1 0 0 0 0 0];

% R is the measurement noise covariance. Generally R and sigma_meas can
% vary between samples. The elements represent variance magnitude.
d.sigma_meas = s.sigma_meas;
d.R = s.sigma_meas^2;