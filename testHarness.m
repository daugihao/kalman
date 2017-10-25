close all
clear

%% Set General Parameters %%%%%%%%%%%%%
env = 4;
NSamples=1000;
dt = 0.01;

%% Ground Truth & Model %%%%%%%%%%%%%%%
switch env
    case 1
        s = train_constVel(NSamples,dt);
        d = model_train_constVelKF(s,dt);
    case 2
        s = train_constAcc(NSamples,dt);
        d = model_train_constAccKF(s,dt);
    case 3
        s = sprungBlocks(NSamples,dt);
        d = model_sprungBlocksKF(s,dt);
    case 4
        s = train_nonLinear(NSamples,dt);
        d = model_train_nonLinearEKF(s,dt);
    case 5
        s = train_nonLinear(NSamples,dt);
        d = model_train_nonLinearUKF(s,dt);
    otherwise
        error(['Selected environment number does not exist: ' num2str(env) '!']);
end

%% Kalman iteration %%%%%%%%%%%%%%%%%%%
if strcmp(d.typeString,'Unscented KF')
    d.Xpred = zeros(size(d.X));
    for k=2:NSamples+1
        % UKF prediction step
        [d.Xpred(:,k),d.P] = ukf_predict1(d.X(:,k-1),d.P,d.predModel,d.Q,d.f_param,d.alpha,d.beta,d.kappa);
        % UKF update step
        [d.X(:,k),d.P] = ukf_update1(d.Xpred(:,k),d.P,s.Y(:,k),d.measModel,d.R,d.h_param,d.alpha,d.beta,d.kappa);
    end
elseif strcmp(d.typeString,'Extended KF')
    for k=2:NSamples+1
        % Compute the Jacobian to obtain the linearised state transition matrix
        d.F = double(subs(d.J,d.X(1,k-1)));
        
        % Compute the predicted mean, d.X1
        d.X1 = d.F * d.X(:,k-1);
        % Compute the predicted covariance matrix, d.P1
        d.P1 = (d.F * d.P * d.F') + d.Q;

        % Compute the predicted measurement, d.Y1
        d.Y1 = d.H * d.X1;
        % Compute the innovation covariance matrix, d.S
        S = (d.H * d.P1 * d.H') + d.R;
        % Compute the Kalman gain (K large -> more weight goes to measurement)
        K = (d.P1 * d.H' / S);

        % Compute the posterior mean, d.X
        d.X(:,k) = d.X1 + (K * (s.Y(:,k) - d.Y1));
        % Compute the covariance matrix, d.P
        d.P = d.P1 - (K * d.H * d.P1);
    end
elseif strcmp(d.typeString,'Linear KF')
    for k=2:NSamples+1
        % Compute the predicted mean, d.X1
        d.X1 = d.F * d.X(:,k-1);
        % Compute the predicted covariance matrix, d.P1
        d.P1 = (d.F * d.P * d.F') + d.Q;

        % Compute the predicted measurement, d.Y1
        d.Y1 = d.H * d.X1;
        % Compute the innovation covariance matrix, d.S
        S = (d.H * d.P1 * d.H') + d.R;
        % Compute the Kalman gain (K large -> more weight goes to measurement)
        K = (d.P1 * d.H' / S);

        % Compute the posterior mean, d.X
        d.X(:,k) = d.X1 + (K * (s.Y(:,k) - d.Y1));
        % Compute the covariance matrix, d.P
        d.P = d.P1 - (K * d.H * d.P1);
    end
else
    error(['Selected model does not exist: ' d.typeString '!']);
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
    t = title([s.modelString ': ' d.typeString]);
    set(t, 'Interpreter', 'none');
end