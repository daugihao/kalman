close all
clear

%% Set General Parameters %%%%%%%%%%%%%
env = 3;
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
    case 3
        s = sprungBlocks(NSamples,dt);
        d = model_sprungBlocks(s,dt);
    otherwise
        error('Selected environment does not exist!');
end

%% Kalman iteration %%%%%%%%%%%%%%%%%%%
for k=2:NSamples+1
    % Compute the predicted mean, d.X1
    d.X1 = d.F * d.X(:,k-1);
    % Compute the predicted covarians matrix, d.P1
    d.P1 = (d.F * d.P * d.F') + d.Q;
    
    % Compute the predicted measurement, d.Y1
    d.Y1 = d.H * d.X1;
    % Compute the innovation covariance matrix, d.S
    S = (d.H * d.P1 * d.H') + d.R;
    % Compute the Kalman gain (K large -> more weight goes to measurement)
    K = (d.P1 * d.H' / S);
    
    % Compute the posterior mean, d.X
    d.X(:,k) = d.X1 + (K * (s.Y(k) - d.Y1));
    % Compute the covariance matrix, d.P
    d.P = d.P1 - (K * d.H * d.P1);
end

%% Plot resulting graphs %%%%%%%%%%%%%%
for i = 1:s.NState
    figure;
    plot(s.t,s.X(i,:),'m');
    hold on; grid on;
    plot(s.t,d.X(i,:),'k');
    title(['Estimation Results: ' s.stateString{i}]);
    xlabel(s.tString);
    ylabel(s.stateString{i});
    legend('True Value','Estimated Value');
end