function f = RMSEclosest_R_a(param, config)
% This function is based on "Robust Point Set Registration Using Gaussian 
% Mixture Models" by Jian, B. and Vemuri, B.C., 2010. Their implementation
% can be seen https://github.com/bing-jian/gmmreg.

% obtain the necessary parameter
model_a = config.model_a;
scene_a = config.scene_a;

% extract the 6 vector param to contruct R and t
R = eul2rotm([param(1) param(2) param(3)], 'ZYX');
t = [param(4) param(5) param(6)];
transformed_modelA = (R * model_a' + t')';

% amode
[~, nearest_dist] = knnsearch(scene_a, transformed_modelA);
f = mean(nearest_dist);

end