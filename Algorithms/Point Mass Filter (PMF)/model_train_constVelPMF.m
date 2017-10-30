function [d] = model_train_constVelPMF(s,dt)
% MODEL_TRAIN_CONSTVEL Model setup

    d.typeString = 'Point Mass Filter';

    %% Select grid points, x^i
    d.grid.range1 = [-0.01 0.1];
    d.grid.discret1 = 30;
    d.grid.step1 = (d.grid.range1(2)-d.grid.range1(1))/(d.grid.discret1-1);
    d.grid.range2 = [9 11];
    d.grid.discret2 = 30;
    d.grid.step2 = (d.grid.range2(2)-d.grid.range2(1))/(d.grid.discret2-1);
    
    d.x = zeros(d.grid.discret1*d.grid.discret2,s.NState);

    %% Evenly space grid points within the desired range
    for i = 1:d.grid.discret1
        for j = 1:d.grid.discret2
            d.x(((i-1)*d.grid.discret1)+j,2) = d.grid.range2(1) + (j-1)*d.grid.step2;
            d.x(((i-1)*d.grid.discret1)+j,1) = d.grid.range1(1) + (i-1)*d.grid.step1;
        end
    end
    
    %% Initialise grid weightings, w^i
    d.w = zeros(d.grid.discret1*d.grid.discret2,1);
    d.w(106) = 1;
    d.w1 = d.w;
    
    d.wprev = ones(size(d.w))./(d.grid.discret1*d.grid.discret2);
    
    %% State estimate initialisation
    d.X = zeros(s.NState,length(s.t));
    d.X(1:s.NState,1) = sum(d.w.*d.x);
    
    %% Measurement matrix
    d.H = [1 0];
    
    %% Prediction model
    d.predModel = @model_train_constVelPMF_predModel;
    d.measModel = @model_train_constVelPMF_measModel;
    d.stateTrans = @model_train_constVelPMF_stateTrans;
    

end