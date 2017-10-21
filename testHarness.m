close all
clear

%% Set General Parameters %%%%%%%%%%%%%
NSamples=100;
dt = 0.1;

%% Ground Truth %%%%%%%%%%%%%%%%%%%%%%%
s = train_constVel(NSamples,dt);

%% System Model %%%%%%%%%%%%%%%%%%%%%%%
d = model_train_constVel(s,dt);

%% Kalman iteration %%%%%%%%%%%%%%%%%%%
% Buffers for later display
Xk_buffer = zeros(2,NSamples+1);
Xk_buffer(:,1) = d.Xk_prev;
Z_buffer = zeros(1,NSamples+1);

for k=1:NSamples
    
    % Z is the measurement vector. In our
    % case, Z = TrueData + RandomGaussianNoise
    Z = s.X(k+1,1)+d.sigma_meas*randn;
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
plot(s.t,s.X(:,1),'g');
hold on;
plot(s.t,Z_buffer,'c');
plot(s.t,Xk_buffer(1,:),'m');
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