close all; clear;

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Tibia_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = (ptCloud.Points - ptCloud_centroid)';

% display the femure bone
% figure1 = figure('Name', 'Bone', 'Position', [0 -100 400 900]);
figure1 = figure(1);
figure1.WindowState = 'maximized';
axes1 = axes('Parent', figure1);
plot3( axes1, ...
       U_breve(1,:), ...
       U_breve(2,:), ...
       U_breve(3,:), ...
       '.', 'Color', [0.7 0.7 0.7], ...
       'MarkerSize', 0.1, ...
       'Tag', 'plot_bone_full');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; hold on;

filename = 'amode_tibia_15_a';
path = 'amode_accessible_sim3';
load(strcat(path, filesep, filename, '.mat'));
U = vertcat(amode_all.Position);
plot3( axes1, ...
       U(:,1), ...
       U(:,2), ...
       U(:,3), ...
       'or', 'MarkerFaceColor', 'r', ...
       'Tag', 'plot_bone_full');
   

%% Simulate Search Space

% obtain all combination of z rotation and translation
range = 10;
step  = 1;
r_z   = (-range:step:range);
% t_z   = (-range/ptCloud_scale:step/ptCloud_scale:range/ptCloud_scale);
t_z = 0;
% change z rotation to rotation matrix
rs = [ r_z', zeros(length(r_z), 2) ];
Rs = eul2rotm(deg2rad(rs), 'ZYX');
% change z translation to translation vector
ts = [ zeros(2, length(t_z)); t_z];

%% Simulate Noise

% add isotropic zero-mean gaussian noise to U, simulating noise measurement
current_noise = 3;
random_noise  = -current_noise/ptCloud_scale + (current_noise/ptCloud_scale + current_noise/ptCloud_scale)*rand(3,size(U, 2));
model_ptCloud = (U + random_noise)';

%% Rz-tz Animation

for current_z = 1:length(r_z)
    for current_t = 1:length(t_z)
        
        % transform Ŭ with respected transformation
        U_breve_prime = Rs(:,:,current_z) * U_breve + ts(:,current_t);
        scene_ptCloud = U_breve_prime;
        
        delete(findobj('Tag', 'plot_bone_transformed'));
        plot3( axes1, ...
               scene_ptCloud(1,:), ...
               scene_ptCloud(2,:), ...
               scene_ptCloud(3,:), ...
               '.g', 'MarkerSize', 0.1, ...
               'Tag', 'plot_bone_transformed');
        
        drawnow;
    end
end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    