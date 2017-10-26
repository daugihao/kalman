function [s] = train_constVel(NSamples,dt)
% TRAIN_CONSTVEL Environment with two states (position and velocity)
% Model has a constant velocity dynamic

%s.X = [ position;
%       velocity];

    s.modelString = 'train_constVel';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'Position (m)';
    s.stateString{2} = 'Velocity (m/s)';
    s.NState = length(s.stateString);
    
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    % Velocity vector (state 2)
    s.X(2,:) = 10*ones(size(s.t));
    % Position vector (state 1)
    s.X(1,:) = s.X(2,:).*s.t;
    
    % U is the input disturbance vector.
    s.B = [0 0]';
    [s.NStateIn, ~] = size(s.B);
    
    s.sigma_cont = 0.00;
    s.X = s.X + (s.B.*s.X + s.sigma_cont*randn(s.NStateIn,length(s.t)) + 0.01);

    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0];
    [s.NStateOut, ~] = size(s.H);
    
    s.sigma_meas = 1;
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end