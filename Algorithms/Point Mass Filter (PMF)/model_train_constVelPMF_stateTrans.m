function [y] = model_train_constVelPMF_stateTrans(x,dt)
   F = [1 dt;
        0  1];
   y = F*x;
end