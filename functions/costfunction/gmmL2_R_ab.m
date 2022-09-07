function [f,g] = gmmL2_R_ab(param, config)
% This function is based on "Robust Point Set Registration Using Gaussian 
% Mixture Models" by Jian, B. and Vemuri, B.C., 2010. Their implementation
% can be seen https://github.com/bing-jian/gmmreg.

% obtain the necessary parameter
model_a = config.model_a;
scene_a = config.scene_a;
scale_a = config.scale_a;
model_b = config.model_b;
scene_b = config.scene_b;
scale_b = config.scale_b;
alpha   = config.alpha;

% extract the 6 vector param to contruct R and t
R = eul2rotm([param(1) param(2) param(3)], 'ZYX');
t = [param(4) param(5) param(6)];
transformed_modelA = (R * model_a' + t')';
transformed_modelB = (R * model_b' + t')';

% amode
[f_a,~] =  GaussTransform(transformed_modelA, scene_a, scale_a);
% bmode
[f_b,~] =  GaussTransform(transformed_modelB, scene_b, scale_b);
% fusion
f = -(f_a + (alpha * f_b));

end