close all; clear all;
addpath('..\..\functions\geometry');

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Femur_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = ptCloud.Points - ptCloud_centroid;
U_breve_normals  = STLVertexNormals(ptCloud.ConnectivityList, ptCloud.Points);

% display the femure bone
figure1 = figure('Name', 'Bone');
figure1.WindowState = 'maximized';
axes1 = axes('Parent', figure1);
% axes1 = subplot(3, 4, [1 5 9]);
plot3( axes1, ...
       U_breve(:,1), ...
       U_breve(:,2), ...
       U_breve(:,3), ...
       '.', 'Color', [0.7 0.7 0.7], ...
       'MarkerSize', 0.1, ...
       'Tag', 'plot_bone_full');
xlabel(axes1, 'X'); ylabel(axes1, 'Y'); zlabel(axes1, 'Z');
grid(axes1, 'on'); axis(axes1, 'equal'); hold(axes1, 'on');

% n_step    = 30;
n_step    = 40;
z_steps   = linspace(max(U_breve(:,3)), min(U_breve(:,3)), n_step+2);
z_planes  = [ repmat( [ 0 0 1 ], n_step, 1), z_steps(2:end-1)' ];
threshold = 0.00075;


figure2 = figure('Name', 'Slices');
figure2.WindowState = 'maximized';
% subplot_axes = [1,2,3,4,5,  6,7,8,9,10,  11,12,13];
subplot_axes = [1,2,3,4,  10,11,12,13,14,15,16,17,18  19,20,21,22,23];
subplot_idx  = 1;

% z_plane_selection = [2,3,4,  6,8,10,12,14,  18,19];
z_plane_selection = [2,3,4,5,  14,16,18,20,22,24,26,28,30, 36,37,38,39,40];

for i=1:length(z_plane_selection)
    
    current_zplane = z_planes(z_plane_selection(i),:);
    
    distance_pointplane = distancePoint2Plane(U_breve, current_zplane');
    condition = ( distance_pointplane < threshold ) & ( distance_pointplane > -threshold );
    selected_U_breve = U_breve(condition, :);
    
    plot3( axes1, ...
           selected_U_breve(:,1), ...
           selected_U_breve(:,2), ...
           selected_U_breve(:,3), ...
           '.r', 'MarkerSize', 0.1, ...
           'Tag', 'plot_selectedUbreve');
       
	axes2 = subplot(3, 9, subplot_axes(subplot_idx), 'Parent', figure2);
    plot3( axes2, ...
           selected_U_breve(:,1), ...
           selected_U_breve(:,2), ...
           selected_U_breve(:,3), ...
           '.r', ...
           'Tag', 'plot_selectedUbreve_single');
    xlabel(axes2, 'X'); ylabel(axes2, 'Y'); zlabel(axes2, 'Z');
    grid(axes2, 'on'); axis(axes2, 'equal'); hold(axes2, 'on');
    view(axes2, 0,90);
	

    subplot_idx = subplot_idx+1;
end