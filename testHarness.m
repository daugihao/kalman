close all
clear

%% Set General Parameters %%%%%%%%%%%%%
env = 1;
NSamples=1000;
dt = 0.01;

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
Xk_buffer = zeros(s.NState,NSamples+1);
Xk_buffer(:,1) = d.Xk_prev;
Z_buffer = zeros(1,NSamples+1);

for k=1:NSamples
    
    % Z is the measurement vector. In our
    % case, Z = TrueData + RandomGaussianNoise
    Z = s.Z(k);
    Z_buffer(k+1) = Z;
    
    % Kalman iteration
    P1 = d.Phi*d.P*d.Phi' + d.Q;
    S = d.M*P1*d.M' + d.R;
    
    % K is Kalman gain. If K is large, more weight goes to the measurement.
    % If K is low, more weight goes to the model prediction.
    K = P1*d.M'*inv(S);
    d.P = P1 - K*d.M*P1;
    
    Xk = d.Phi*d.Xk_prev + K*(Z-d.M*d.Phi*d.Xk_prev);
    Xk_buffer(:,k+1) = Xk;
    
    % For the next iteration
    d.Xk_prev = Xk; 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot resulting graphs %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Position analysis %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
plot(s.t,s.X(:,1),'m');
hold on;
plot(s.t,Z_buffer,'c');
plot(s.t,Xk_buffer(1,:),'k');
title('Position estimation results');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True position','Measurements','Kalman estimated displacement');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity analysis %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The instantaneous velocity as derived from 2 consecutive position
% measurements
InstantaneousVelocity = [0 (Z_buffer(2:NSamples+1)-Z_buffer(1:NSamples))/dt];

% The instantaneous velocity as derived from running average with a window
% of 5 samples from instantaneous velocity
WindowSize = 5;
InstantaneousVelocityRunningAverage = filter(ones(1,WindowSize)/WindowSize,1,InstantaneousVelocity);

figure;
plot(s.t,s.X(:,2),'m');
hold on;
plot(s.t,InstantaneousVelocity,'g');
plot(s.t,InstantaneousVelocityRunningAverage,'c');
plot(s.t,Xk_buffer(2,:),'k');
title('Velocity estimation results');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True velocity','Estimated velocity by raw consecutive samples','Estimated velocity by running average','Estimated velocity by Kalman filter');