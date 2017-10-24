function [d] = model_train_nonLinearUKF(s,dt)
% MODEL_TRAIN_NONLINEAR Model setup

d.typeString = 'Unscented KF';

d.measModel = @model_train_nonLinearMeasurementModel;
d.predModel = @model_train_nonLinearPredictionModel;

d.X = zeros(s.NState,length(s.t));

% Previous state (initial guess)
d.X(:,1) = [1; 
    5.0;
    -cos(1)];

% The error matrix (or the confidence matrix): P gives the confidence in
% the initial estimate. A low value indicates that the initial state should
% be trusted (it represents the level of error associated with that
% measurement).
d.P = [1e-6 0 0;
       0 1e-6 0;
         0 0 1e-6];

% Q is the process noise covariance. It represents the amount of
% uncertainty in the model. In our case, we arbitrarily assume that the model is perfect (no
% acceleration allowed for the train, or in other words - any acceleration
% is considered to be a noise). The elements represent variance magnitude.
d.Q = [0 0 0;
     0 0 0;
     0 0 0];
 
% R is the measurement noise covariance. Generally R and sigma_meas can
% vary between samples. The elements represent variance magnitude.
d.sigma_meas = s.sigma_meas;
d.R = s.sigma_meas^2;

%Unscented transform parameters. Using values recommended in "The Unscented
%Kalman Filter for Nonlinear Estimation", Wan and van der Merwe, 2000.
d.alpha = 1e-3; %parameter determining spread of sigma points about mean
d.beta = 2; %parameter for incorporating prior knowledge of distribution. 2 is optimal for gaussian distributions.
d.kappa = 0; %"Secondary scaling parameter usually set to 0"

d.f_param = dt;
d.h_param = [];