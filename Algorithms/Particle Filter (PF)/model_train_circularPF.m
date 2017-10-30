function [d] = model_train_circularPF(s,dt)
% MODEL_CIRCULAR Model setup

    d.typeString = 'Particle Filter';
    
    %% State estimate initialisation
    d.X = zeros(s.NState,length(s.t));
    d.X(1:s.NState,1) = [1.1; 0];
    
    %% Select sample points, x^i
    d.samples.number = 50;   
    d.x = zeros(d.samples.number,s.NState);

    %% Initialisation of sample point weightings
    for i = 1:length(d.x)
        for j = 1:s.NState
            d.x(i,j) = d.X(j,1) + sqrt(0.1) * randn;
        end
    end
    
    d.w = zeros(d.samples.number,1);
    d.wprev = ones(size(d.w))./d.samples.number;
    
    %% Measurement matrix
    d.H = [1 0];
    
    %% Prediction model
    d.predModel = @model_train_circularPF_predModel;
    d.measModel = @model_train_circularPF_measModel;
    d.stateTrans = @model_train_circularPF_stateTrans;

end