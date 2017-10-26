function y = model_train_nonLinearEKFequation(x,dt)
%Will Reynolds 24/10/2017
%UKF non-linear equations for non-linear train model
y = zeros(size(x));

y(1) = x(1) + x(2)*dt;
y(2) = x(2) + x(3)*dt;
y(3) = -sin(x(1));

end