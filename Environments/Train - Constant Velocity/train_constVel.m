function [s] = train_constVel(NSamples,dt)
% TRAIN_CONSTVEL Environment with two states (position and velocity)
% Model has a constant velocity dynamic

%s.X = [ position;
%       velocity];

    s.t=(0:dt:dt*NSamples)';
    s.NState = 2;
    
    % Initialise the state array
    s.X = zeros(length(s.t),s.NState);
    % Velocity vector (state 2)
    s.X(:,2) = 10*ones(size(s.t));
    % Position vector (state 1)
    s.X(:,1) = s.X(:,2).*s.t;

    % Z is the measurement vector. In our case, Z = TrueData + RandomGaussianNoise
    s.sigma_meas = 1; % 1 m/sec
    s.Y = s.X(:,1)+s.sigma_meas*randn(size(s.t));

end