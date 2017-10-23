close all
clear

%% Set General Parameters %%%%%%%%%%%%%
env = 2;
NSamples=100;
dt = 0.1;

%% Ground Truth & Model %%%%%%%%%%%%%%%
switch env
    case 1
        s = train_constVel(NSamples,dt);
        d = model_train_constVel(s,dt);
    case 2
        s = train_constAcc(NSamples,dt);
        d = model_train_constAcc(s,dt);
    otherwise
        error('Selected environment does not exist!');
end

%% Kalman iteration %%%%%%%%%%%%%%%%%%%
% Buffers for later display

for k=2:NSamples+1
    % Kalman iteration
    d.P1 = (d.F * d.P * d.F') + d.Q;
    S = (d.H * d.P1 * d.H') + d.R;
    
    % K is Kalman gain. If K is large, more weight goes to the measurement.
    % If K is low, more weight goes to the model prediction.
    K = (d.P1 * d.H' / S);
    d.P = d.P1 - (K * d.H * d.P1);
    
    d.X(:,k) = (d.F * d.X(:,k-1)) + (K * (s.Y(k) - (d.H * d.F * d.X(:,k-1))));
end

%% Plot resulting graphs %%%%%%%%%%%%%%
figure;
plot(s.t,s.X(:,1),'m');
hold on;
plot(s.t,s.Y,'c');
plot(s.t,d.X(1,:),'k');
title('Position estimation results');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True position','Measurements','Kalman estimated displacement');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity analysis %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The instantaneous velocity as derived from 2 consecutive position
% measurements
InstantaneousVelocity = [0 (s.Y(2:NSamples+1)-s.Y(1:NSamples))'/dt];

% The instantaneous velocity as derived from running average with a window
% of 5 samples from instantaneous velocity
WindowSize = 5;
InstantaneousVelocityRunningAverage = filter(ones(1,WindowSize)/WindowSize,1,InstantaneousVelocity);

figure;
plot(s.t,s.X(:,2),'m');
hold on;
plot(s.t,InstantaneousVelocity,'g');
plot(s.t,InstantaneousVelocityRunningAverage,'c');
plot(s.t,d.X(2,:),'k');
title('Velocity estimation results');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True velocity','Estimated velocity by raw consecutive samples','Estimated velocity by running average','Estimated velocity by Kalman filter');