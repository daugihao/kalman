function [s] = sprungBlocks(NSamples,dt)
% SPRUNG_BLOCKS Environment with six states 2x(position, velocity and acceleration)

%s.X = [ position_1;
%       velocity_1;
%       acceleration_1;
%       position_2;
%       velocity_2;
%       acceleration_2];

    s.modelString = 'sprungBlocks';

    s.t=(0:dt:dt*NSamples);
    s.tString = 'Time (s)';
    
    s.stateString{1} = 'Position - Block 1 (m)';
    s.stateString{2} = 'Velocity - Block 1 (m/s)';
    s.stateString{3} = 'Acceleration - Block 1 (m/s^2)';
    s.stateString{4} = 'Position - Block 2 (m)';
    s.stateString{5} = 'Velocity - Block 2 (m/s)';
    s.stateString{6} = 'Acceleration - Block 2 (m/s^2)';
    s.NState = length(s.stateString);
    
    % Model parameters
    k = 1; % Spring stiffness, N/m
    m = 1; % Carriages masses, kg
    
    % Initialise the state array
    s.X = zeros(s.NState,length(s.t));
    
    s.X(1,1) = 0;
    s.X(2,1) = 0;
    s.X(3,1) = k/m;
    s.X(4,1) = 1;
    s.X(5,1) = 0;
    s.X(6,1) = -k/m;
    
    s.F = [1 dt 0 0 0 0;
       0  1 dt 0 0 0;
       -k/m 0 0 k/m 0 0;
       0 0 0 1 dt 0;
       0 0 0 0 1 dt
       k/m 0 0 -k/m 0 0];
    
    for i = 2:length(s.t)
        s.X(:,i) = s.F*s.X(:,i-1);
    end

    % Y is the measurement vector. In our case, Y = TrueData + RandomGaussianNoise
    s.H = [1 0 0 0 0 0];
    [s.NStateOut, ~] = size(s.H);
    
    s.sigma_meas = 1;
    s.Y = s.H*s.X + s.sigma_meas*randn(s.NStateOut,length(s.t));

end