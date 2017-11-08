function [s] = train_constAcc(NSamples,dt)
% TRAIN_CONSTACC Environment with three states (position, velocity and acceleration)
% Model has a constant acceleration dynamic

%s.X = [ position;
%       velocity;
%       acceleration];

    %% General Information
    s.modelString = 'train_constAcc';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'Position (m)';
    s.stateString{2} = 'Velocity (m/s)';
    s.stateString{3} = 'Acceleration (m/s^2)';
    s.NState = length(s.stateString);
    
    %% State Information 
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    % Acceleration vector (state 3)
    s.X(3,:) = 1.0*ones(size(s.t));
    % Velocity vector (state 2)
    s.X(2,:) = s.X(3,:).*s.t;
    % Position vector (state 1)
    s.X(1,1) = 0;
    for i = 2:length(s.X(2,:))
        s.X(1,i) = s.X(1,i-1) + s.X(2,i)*dt;
    end

    %% Measurement Information
    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0 0];
    [s.NStateOut, ~] = size(s.H);
    
    s.sigma_meas = 1;
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end