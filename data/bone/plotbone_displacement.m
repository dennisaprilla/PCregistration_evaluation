close all; clear;

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Tibia_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = (ptCloud.Points - ptCloud_centroid)';

% display the femure bone
figure1 = figure('Name', 'Bone');
figure1.WindowState  = 'maximized';
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
view(axes1, [15, 85]);

filename = 'amode_tibia_15';
path = 'amode_accessible_sim2';
load(strcat(path, filesep, filename, '.mat'));
U = vertcat(amode_all.Position)';
plot3( axes1, ...
       U(1,:), ...
       U(2,:), ...
       U(3,:), ...
       'or', 'MarkerFaceColor', 'r', ...
       'Tag', 'plot_bone_full');

range = 10;
r_z   = 10;
% r_z   = (range:-1:0);
rs    = [ r_z', zeros(length(r_z), 2) ];
Rs    = eul2rotm(deg2rad(rs), 'ZYX');

mean_displacements = zeros(1, range);

for current_z = 1:length(r_z)


    U_breve_prime = Rs(:,:,current_z) * U_breve;
    U_prime = Rs(:,:,current_z) * U; 
    
    %
    delete(findobj('Tag', 'plot_bonetransformed'));
    plot3( axes1, ...
           U_breve_prime(1,:), ...
           U_breve_prime(2,:), ...
           U_breve_prime(3,:), ...
           '.g', ...
           'MarkerSize', 0.1, ...
           'Tag', 'plot_bonetransformed');
    plot3( axes1, ...
           U_prime(1,:), ...
           U_prime(2,:), ...
           U_prime(3,:), ...
           'og', 'MarkerFaceColor', 'g', ...
           'Tag', 'plot_bonetransformed');
    
    for i=1:size(U,2)
        temp_pointPair = [U(:,i), U_prime(:,i)]';
        plot3( axes1, temp_pointPair(:,1), temp_pointPair(:,2), temp_pointPair(:,3), ...
               '-m', 'Tag', 'plot_bonetransformed');
    end    
    %
    
    % mean_displacements(current_z) = mean(vecnorm((U_prime - U), 2, 1));
    
end

%{
% figure;
% title('Mean displacements')
% plot(mean_displacements * ptCloud_scale, r_z, 'r-o', 'MarkerFaceColor', 'r');
% xline(1,'--g',{'Displacement 1'});
% xline(2,'--r',{'Displacement 2'});
% set(gca, 'YGrid', 'on');
% xlabel('Mean Displacement (mm)');
% ylabel('Z-Degree');
% 
% figure;
% title('Mean displacements')
% plot(r_z, mean_displacements * ptCloud_scale, 'r-o', 'MarkerFaceColor', 'r');
% yline(1,'--g',{'Noise 1', 'Equivalent Threshold'});
% yline(2,'--r',{'Noise 2', 'Equivalent Threshold'});
% set(gca, 'XGrid', 'on');
% xlabel('Z-Degree');
% ylabel('Mean Displacement (mm)');
%}






