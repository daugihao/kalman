function [s] = train_constAcc(NSamples,dt)
% TRAIN_CONSTACC Environment with three states (position, velocity and acceleration)
% Model has a constant acceleration dynamic

%s.X = [ position;
%       velocity];

    s.t=(0:dt:dt*NSamples)';
    s.NState = 3;
    
    % Initialise the state array
    s.X = zeros(length(s.t),s.NState);
    % Acceleration vector (state 3)
    s.X(:,3) = 1.0*ones(size(s.t));
    % Velocity vector (state 2)
    s.X(:,2) = s.X(:,3).*s.t;
    % Position vector (state 1)
    s.X(1,1) = 0;
    for i = 2:length(s.X(:,2))
        s.X(i,1) = s.X(i-1,1) + s.X(i,2)*dt;
    end

    % Z is the measurement vector. In our case, Z = TrueData + RandomGaussianNoise
    s.sigma_meas = 1; % 1 m/sec
    s.Y = s.X(:,1)+s.sigma_meas*randn(size(s.t));

end