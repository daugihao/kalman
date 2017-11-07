function [s] = train_constVel(NSamples,dt)
% TRAIN_CONSTVEL Environment with two states (position and velocity)
% Model has a constant velocity dynamic

%s.X = [ position;
%       velocity];

    %% General Information
    s.modelString = 'train_constVel';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'Position (m)';
    s.stateString{2} = 'Velocity (m/s)';
    s.NState = length(s.stateString);
    
    %% State Information 
    % Define process noise variance
    s.sigma_proc = 0.01; 
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    % Velocity vector (state 2)
    s.X(2,:) = 10*ones(size(s.t)) + s.sigma_proc*randn(1,length(s.t));
    % Position vector (state 1)
    s.X(1,:) = s.X(2,:).*s.t;

    %% Measurement Information
    % Define measurement noise variance
    s.sigma_meas = 0.1;
    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0];
    [s.NStateOut, ~] = size(s.H);
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end