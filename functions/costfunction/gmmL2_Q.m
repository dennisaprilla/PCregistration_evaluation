function [f,g] = gmmL2_Q(param,config)
% This function is based on "Robust Point Set Registration Using Gaussian 
% Mixture Models" by Jian, B. and Vemuri, B.C., 2010. Their implementation
% can be seen https://github.com/bing-jian/gmmreg.

% i only took the case for rigid3d because it is the only relevant
% costfunction for our case.

model = config.model;
scene = config.scene;
scale = config.scale;
[transformed_model] = transform_pointset(model, motion, param);

[f,grad] = rigid_costfunc(transformed_model, scene, scale);
[~,gq]   = quaternion2rotation(param(1:4));

grad = grad';
gm   = grad*model; 

g(1) = sum(sum(gm.*gq{1}));
g(2) = sum(sum(gm.*gq{2}));
g(3) = sum(sum(gm.*gq{3}));
g(4) = sum(sum(gm.*gq{4}));        
g(5) = sum(grad(1,:));
g(6) = sum(grad(2,:));
g(7) = sum(grad(3,:));
end

function [f, g] = rigid_costfunc(A, B, scale)
[f, g] =  GaussTransform(A,B,scale);
f = -f; g = -g;
end