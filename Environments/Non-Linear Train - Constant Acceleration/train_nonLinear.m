function [s] = train_nonLinear(NSamples,dt)
% TRAIN_NONLINEAR Environment with three states (position, velocity and acceleration)
% Model has a non-linear acceleration dynamic

%s.X = [ position;
%       velocity];

    s.modelString = 'train_nonLinear';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'Position (m)';
    s.stateString{2} = 'Velocity (m/s)';
    s.stateString{3} = 'Acceleration (m/s^2)';
    s.NState = length(s.stateString);
    
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    
    % Position vector (state 1)
    s.X(1,1) = 1;
    % Velocity vector (state 2)
    s.X(2,1) = 5;
    % Acceleration vector (state 3)
    s.X(3,1) = -cos(s.X(1,1));
    
    for i = 2:NSamples+1
        s.X(1,i) = s.X(1,i-1) + s.X(2,i-1)*dt;
        s.X(2,i) = s.X(2,i-1) + s.X(3,i-1)*dt;
        s.X(3,i) = -sin(s.X(1,i-1));        
    end

    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0 0];
    [s.NStateOut, ~] = size(s.H);
    
    s.sigma_meas = 1;
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end