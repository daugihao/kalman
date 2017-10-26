function y = model_train_nonLinearPredictionModel(x,params)
%Will Reynolds 24/10/2017
%UKF prediction model for non-linear train model
y = zeros(size(x));
dt = params(1);

y(1) = x(1) + x(2)*dt;
y(2) = x(2) + x(3)*dt;
y(3) = -sin(x(1));

end