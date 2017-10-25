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

    syms x1 x2 x3
    s.J = jacobian([x1 + x2*dt, x2 + x3*dt, -sin(x1)], [x1, x2, x3]);
    
    for i = 2:NSamples+1
        s.X(:,i) = subs(s.J,s.X(1,i-1))*s.X(:,i-1);
    end

    % Z is the measurement vector. In our case, Z = TrueData + RandomGaussianNoise
    s.sigma_meas = 1;
    s.Y = s.X(1,:)+s.sigma_meas*randn(size(s.t));

end