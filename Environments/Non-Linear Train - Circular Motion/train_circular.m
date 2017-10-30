function [s] = train_circular(NSamples,dt)
% TRAIN_CIRCULAR Environment with two states (x-position and y-position)
% Model has a constant angular velocity dynamic, but is represented in
% Cartesian

    %s.X = [ x-position;
    %       y-position];

    s.modelString = 'train_circular';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'X-Position (m)';
    s.stateString{2} = 'Y-Position (m)';
    s.NState = length(s.stateString);
    
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    % Velocity vector (state 2)
    s.X(1,:) = cos(s.t*10);
    % Position vector (state 1)
    s.X(2,:) = sin(s.t*10);

    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0];
    [s.NStateOut, ~] = size(s.H);
    
    s.sigma_meas = 0.01;
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end