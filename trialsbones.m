clc; clear; close all;
%% Reading simulation config from .INI file

% download the function here
% https://nl.mathworks.com/matlabcentral/fileexchange/17177-ini2struct
addpath('functions\external\ini2struct');
simconfig = ini2struct('simconf.ini');

%% Write the parameter from .INI file

% path to data
path_bone   = simconfig.pathdata.path_bone;
path_amode  = simconfig.pathdata.path_amode;
path_result = simconfig.pathdata.path_result;

% path to project
path_icpnormal = simconfig.pathalgorithm.icpnormal;
path_ukf       = simconfig.pathalgorithm.ukf;
path_cpd       = simconfig.pathalgorithm.cpd;
path_goicp     = simconfig.pathalgorithm.goicp;

% add paths
addpath(path_icpnormal);
addpath(path_ukf);
addpath(genpath(path_cpd));
addpath(path_goicp);

displaybone = logical(str2num(simconfig.simulation.displaybone));

%% Prepare the bone point cloud

% read the point cloud (bone) from STL/PLY file
filename_bonedata = simconfig.simulation.filename_bonedata;
filepath_bonedata = strcat(path_bone, filesep, filename_bonedata, '.stl');
ptCloud           = stlread(filepath_bonedata);
% scale the point cloud in in mm unit
ptCloud_scale     = 1000;
ptCloud_Npoints   = size(ptCloud.Points,1);
ptCloud_centroid  = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve           = (ptCloud.Points - ptCloud_centroid) * ptCloud_scale;
U_breve_hat       = STLVertexNormals(ptCloud.ConnectivityList, ptCloud.Points);

% show figure for sanity check
if(displaybone)
    figure1 = figure('Name', 'Registration in Measurement Coordinate System', 'Position', [0 0 350 780]);
    axes1 = axes('Parent', figure1);
    plot3( axes1, ...
           U_breve(:,1), ...
           U_breve(:,2), ...
           U_breve(:,3), ...
           '.', 'Color', [0.7 0.7 0.7], ...
           'MarkerSize', 0.1, ...
           'Tag', 'plot_Ubreve');
    grid on; axis equal; hold on;
%     quiver3(U_breve(:,1), U_breve(:,2), U_breve(:,3), ...
%             U_breve_normals(:,1), U_breve_normals(:,2), U_breve_normals(:,3));
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

%% Prepare the A-mode measurement simulation

% read the point cloud (A-mode) from the mat file
filename_amodedata = simconfig.simulation.filename_amodedata;
filepath_amodedata = strcat(path_amode, filesep, filename_amodedata, '.mat');
load(filepath_amodedata);

% get the amode
U     = vertcat(amode_all.Position) * ptCloud_scale;
% if the dataset has normal, we can directly load it
if (exist('amode_all_normals', 'var'))
    U_hat = U_breve_hat(vertcat(amode_all.DataIndex), :);
% if not, lets do some estimation
else
    nearest_idx   = knnsearch(U_breve, U);
    U_hat = U_breve_hat(nearest_idx, :);
end

% (for debugging only) show figure for sanity check
if(displaybone)
    plot3( axes1, ...
           U(:,1), ...
           U(:,2), ...
           U(:,3), ...
           'or', 'MarkerFaceColor', 'r', ...
           'Tag', 'plot_U');
    quiver3(U(:,1),     U(:,2),     U(:,3), ...
            U_hat(:,1), U_hat(:,2), U_hat(:,3), 0.1, ...
            'Tag', 'plot_Uhat');
    title('Initial Setup');

    drawnow;
    pause(0.5);
end

%% Simulation Config

noisetype         = simconfig.simulation.noisetype;
noises            = [0, 1.0, 2.0, 3.0];
noisenormal_const = 2;
init_poses        = [10];
n_trials          = str2num(simconfig.simulation.n_trials);

description.algorithm  = simconfig.simulation.algorithm;
description.noises     = noises;
description.init_poses = init_poses;
description.trials     = n_trials;
description.dim_desc   = ["trials", "observation dimensions", "noises", "initial poses"];

trial_number      = str2num(simconfig.simulation.trial_number);
point_number      = str2num(simconfig.simulation.point_number);
filename_result   = sprintf('%s_%d_trials%d.mat', description.algorithm, point_number, trial_number);

GTs               = zeros(n_trials, 6, length(noises), length(init_poses));
estimations       = zeros(n_trials, 6, length(noises), length(init_poses));
errors            = zeros(n_trials, 6, length(noises), length(init_poses));
rmse_measurements = zeros(n_trials, 1, length(noises), length(init_poses));
rmse_trues        = zeros(n_trials, 1, length(noises), length(init_poses));
exec_time         = zeros(n_trials, 1, length(noises), length(init_poses));


%% Simulation Start

for init_pose=1:length(init_poses)
    
max_t     = init_poses(init_pose);
max_theta = init_poses(init_pose);

for noise=1:length(noises)
    
noise_point  = noises(noise);
noise_normal = noise_point * noisenormal_const;

trial = 1;
while (trial <= n_trials)
% for trial=1:n_trials
    fprintf('init pose: %d, noise: %d, trial: %d... ', init_pose, noise, trial);
    
    %% apply some random noise

    % % add isotropic zero-mean gaussian noise to U, simulating noise measurement
    % % uncomment this block if you want to use standard deviation for noise
    if (strcmp(noisetype, 'uniform'))
        random_point   = -(noise_point) + 2*(noise_point) * rand(size(U, 1), 3);
    elseif (strcmp(noisetype, 'isotropic_gaussian'))
        random_point   = mvnrnd( [0 0 0], eye(3)*noise_point, size(U, 1));
    end
    U_noised       = U + random_point;
    
    % if the algorithm specified by user is using normal, we provide the
    % normal calculations
    if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
        U_hat_noised = [];
        for i=1:size(U_hat,1)
            if (strcmp(noisetype, 'uniform'))
                random_normal = -noise_normal + 2*noise_normal * rand(1, 3);
            elseif (strcmp(noisetype, 'isotropic_gaussian'))
                random_normal   = mvnrnd( [0 0 0], eye(3)*noise_normal, 1);
            end
            random_R      = eul2rotm(deg2rad(random_normal), 'ZYX');
            U_hat_noised  = [U_hat_noised; (random_R * U_hat(i,:)')'];
        end
    end
    
    % show figure for sanity check
    if (displaybone)
        delete(findobj('Tag', 'plot_U'));
        delete(findobj('Tag', 'plot_Uhat'));
        plot3( axes1, ...
               U_noised(:,1), ...
               U_noised(:,2), ...
               U_noised(:,3), ...
               'or', ...
               'Tag', 'plot_U_noised');    
        % if the algorithm specified by user is using normal, we provide the
        % normal calculations
        if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
            quiver3(axes1, ...
                    U_noised(:,1),     U_noised(:,2),     U_noised(:,3), ...
                    U_hat_noised(:,1), U_hat_noised(:,2), U_hat_noised(:,3), 0.1, ...
                    'Tag', 'plot_Uhat_noised');
        end
        title('Noise Measurment Added');
        
        drawnow;
        pause(0.5);       
    end
    
    
    %% radom transformation, point selection, and noise
    
    % contruct a arbritary transformation then apply it to Ŭ in order to
    % generate Y̆, the noiseless, complete, fixed dataset.
    random_trans = -max_t     + (max_t -(-max_t))         .* rand(1, 3);
    random_theta = -max_theta + (max_theta -(-max_theta)) .* rand(1, 3);
    random_R     = eul2rotm(deg2rad(random_theta), 'ZYX');
    GT           = [random_trans, random_theta];
    Y_breve      = (random_R * U_breve' + random_trans')';
    
    % if the algorithm specified by user is using normal, we provide the
    % normal calculations
    if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
        Y_breve_hat  = (random_R * U_breve_hat')';
    end
    
    % show figure for sanity check
    if(displaybone)
        plot3( axes1, ...
               Y_breve(:,1), ...
               Y_breve(:,2), ...
               Y_breve(:,3), ...
               '.g', 'MarkerSize', 0.1, ...
               'Tag', 'plot_Ybreve');
        title('Random Transformation Applied');

        drawnow;
        pause(0.5);
    end
    

    %% registration
    
    t_start = tic;
    if (strcmp(description.algorithm, 'icp'))

        % ICP Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U_noised);
        fixed  = pointCloud(Y_breve);
        % register with icp
        [tform, movingReg, icp_rmse] = pcregistericp( moving, ...
                                                      fixed, ...
                                                      'InlierRatio', 1, ...
                                                      'Verbose', false, ...
                                                      'MaxIteration', 50 );
        % change the T form
        T_all   = tform.T';
        % store the rmse
        rmse_measurement = icp_rmse;
        
    elseif (strcmp(description.algorithm, 'icpnormal'))
        
        % ICP normal registration
        moving       = U_noised;
        movingnormal = U_hat_noised * ptCloud_scale;
        fixed        = Y_breve;
        fixednormal  = Y_breve_hat * ptCloud_scale;
        [T_all, icpnormal_rmse] = icpnormal( moving, movingnormal, ...
                                             fixed, fixednormal, ...
                                             U_breve, ...
                                             'iteration', 100, ...
                                             'threshold', 1, ...
                                             'normalratio', 0.05, ...
                                             'ransacdistance', 5, ...
                                             'verbose', false, ...
                                             'display', false);

        % sometimes ransac method in normal icp cant find the inlier, and
        % it will produce error, so redo this loop is that happen
        if (isnan(icpnormal_rmse))
            continue;
        % if there is no error, just do as usual
        else
            rmse_measurement = icpnormal_rmse;  
        end            

    elseif (strcmp(description.algorithm, 'cpdmatlab'))
    
        % CPD Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U_noised);
        fixed  = pcdownsample( pointCloud(Y_breve), 'gridAverage', 3);
        % register with icp
        [tform, movingReg, cpd_rmse] = pcregistercpd( moving, ...
                                                      fixed, ...
                                                      'Transform', 'Rigid', ...
                                                      'OutlierRatio', 0.1, ...
                                                      'MaxIteration', 1000, ...
                                                      'Tolerance', 1e-10, ...
                                                      'verbose', false);
    	% change the T form
    	T_all   = tform.T';
        % store the rmse
        rmse_measurement = cpd_rmse;
        
    elseif (strcmp(description.algorithm, 'cpdmyronenko'))
        
        moving = U_noised;
        fixed  = pcdownsample( pointCloud(Y_breve), 'gridAverage', 3).Location;
        
        % Set the options
        opt.method='rigid'; % use rigid registration
        opt.viz=0;          % show every iteration
        opt.outliers=0.075;   % use 0.6 noise weight

        opt.normalize=0;    % normalize to unit variance and zero mean before registering (default)
        opt.scale=0;        % estimate global scaling too (default)
        opt.rot=1;          % estimate strictly rotational matrix (default)
        opt.corresp=1;      % do not compute the correspondence vector at the end of registration (default).
                            % Can be quite slow for large data sets.

        opt.max_it=1000;    % max number of iterations
        opt.tol=1e-15;      % tolerance
        opt.fgt=0;          % [0,1,2] if > 0, then use FGT.
                            % case 1: FGT with fixing sigma after it gets too small 
                            %         (faster, but the result can be rough)
                            % case 2: FGT, followed by truncated Gaussian approximation 
                            %         (can be quite slow after switching to the truncated kernels, 
                            %         but more accurate than case 1)
                            
        % register with cpd
        T = cpd_register(fixed, moving, opt);
        axis equal;
        % construct the T
        T_all = [T.R, T.t; 0 0 0 1];
        % no rmse reported, so give it NaN
        rmse_measurement = NaN;        
        
    elseif (strcmp(description.algorithm, 'ukf'))

        % UKF Registration
        [T_all, mean_dist, history] = ukf_isotropic_registration( U_noised', Y_breve', U_breve', ...
                                           'threshold', 0.0001, ...
                                           'iteration', 150, ...
                                           'expectednoise', 1.2*noise_point, ...
                                           'sigmaxanneal', 0.98, ...
                                           'sigmaxtrans', 1.0*max_t, ...
                                           'sigmaxtheta', 1.0*max_theta, ...
                                           'bestrmse', true, ...
                                           'verbose', false, ...
                                           'display', false);
        % store the rmse
        rmse_measurement = mean_dist;
        
    elseif (strcmp(description.algorithm, 'ukfnormal'))
        
        % UKF Registration with normals
        movingnormal = U_hat_noised' * ptCloud_scale;
        fixednormal = Y_breve_hat' * ptCloud_scale;
        %{
        [T_all, mean_dist, history] = ukf_isotropic_registration_ex2( U_noised', Y_breve', U_breve', ...
                                           'movingnormal', movingnormal, ...
                                           'fixednormal', fixednormal, ...
                                           'normalratio', 0.05, ...
                                           'threshold', 0.0001, ...
                                           'iteration', 100, ...
                                           'expectednoise', 1.0*noise_normal, ...
                                           'sigmaxanneal', 0.98, ...
                                           'sigmaxtrans', 1.0*max_t, ...
                                           'sigmaxtheta', 1.0*max_theta, ...
                                           'bestrmse', true, ...
                                           'verbose', true, ...
                                           'display', true);
        %}
        [T_all, mean_dist, history] = ukf_isotropic_registration_ex2( U_noised', Y_breve', U_breve', ...
                                           'movingnormal', movingnormal, ...
                                           'fixednormal', fixednormal, ...
                                           'normalratio', 0.025, ...
                                           'threshold', 0.0001, ...
                                           'iteration', 150, ...
                                           'expectednoise', 1.2*noise_normal, ...
                                           'sigmaxanneal', 0.98, ...
                                           'sigmaxtrans', 1.0*max_t, ...
                                           'sigmaxtheta', 1.0*max_theta, ...
                                           'bestrmse', true, ...
                                           'verbose', false, ...
                                           'display', false);
        % store the rmse
        rmse_measurement = mean_dist;

    elseif (strcmp(description.algorithm, 'goicp'))
            
        % GO-ICP Registration
        % normalize everything
        temp = [U_noised; Y_breve];
        scale = max(max(abs(temp)));
        temp = temp ./ scale;
        data = temp(1:size(U_noised, 1), :);
        model = temp(size(U_noised, 1)+1:end, :);
        % store data.txt
        fileID = fopen('data\temp\data.txt','w+');
        fprintf(fileID,'%d\n', size(data, 1));
        fprintf(fileID,'%f %f %f\n', data');
        fclose(fileID);
        % store model.txt
        fileID = fopen('data\temp\model.txt','w+');
        fprintf(fileID,'%d\n',  size(model, 1));
        fprintf(fileID,'%f %f %f\n', model');
        fclose(fileID); 
%         % verify the data
%         model_read = readpoints('data\temp\model.txt');
%         data_read = readpoints('data\temp\data.txt');
%         figure(3);
%         plot3(data_read(1,:), data_read(2,:), data_read(3,:), 'or');
%         hold on; grid on;
%         plot3(model_read(1,:),  model_read(2,:),  model_read(3,:), '.b');
%         hold off; axis equal; title('Initial Pose');
%         break;
        % run GO-ICP
        goicp_exe  = "GoICP_vc2012";
        cmd = sprintf("%s %s %s %d %s %s", ...
                      strcat(path_goicp, filesep, "demo", filesep, goicp_exe), ...
                      "data\temp\model.txt", ...
                      "data\temp\data.txt", ...
                      size(U_noised, 1), ...
                      strcat(path_goicp, filesep, "demo", filesep, "config_modified.txt"), ...
                      "data\temp\output.txt");
        system(cmd);
        % open output file
        file = fopen('data\temp\output.txt', 'r');
        time = fscanf(file, '%f', 1);
        R    = fscanf(file, '%f', [3,3])';
        t    = fscanf(file, '%f', [3,1]) * scale;
        fclose(file);
        % delete the file
        delete('data\temp\data.txt');
        delete('data\temp\model.txt');
        delete('data\temp\output.txt');
        % reformat the T
        T_all = [R, t; 0 0 0 1];
        % no rmse reported, so give it NaN
        rmse_measurement = NaN;
    end
    t_end = toc(t_start);
    fprintf(' (time: %.4fs)\n', t_end);    
    
    %% calculate performance

    t_all      = T_all(1:3, 4);
    R_all      = T_all(1:3, 1:3);
    eul_all    = rad2deg(rotm2eul(R_all, 'ZYX'));
    Uest       = (R_all * U' + t_all)';
    Uest_breve = (R_all * U_breve' + t_all)';
   
    if(displaybone)
        delete(findobj('Tag', 'plot_Ubreve'));
        delete(findobj('Tag', 'plot_U_noised'));
        delete(findobj('Tag', 'plot_Uhat_noised'));
        plot3( axes1, ...
               Uest(:,1), ...
               Uest(:,2), ...
               Uest(:,3), 'or', 'MarkerFaceColor', 'r', ...
               'Tag', 'plot_Uest');
        plot3( axes1, ...
               Uest_breve(:,1), ...
               Uest_breve(:,2), ...
               Uest_breve(:,3), ...
               '.r', 'MarkerSize', 0.1, ...
               'Tag', 'plot_Uest_breve');
    end
    % zlim([100, max(U_breve(:,3))]);
    % legend({'Transformed bone (ground truth position)', 'Synthetic A-mode point (fail registered)', 'Corresponding bone position for registered A-mode'}, 'FontSize', 12);
    
    % store the results
    GTs(trial, :, noise, init_pose)               = GT;
    estimations(trial, :, noise, init_pose)       = [t_all', eul_all];
    errors(trial, :, noise, init_pose)            = diff( [GT; [t_all', eul_all] ], 1, 1);
    rmse_measurements(trial, :, noise, init_pose) = rmse_measurement;
    rmse_trues(trial, :, noise, init_pose)        = mean(sqrt(sum((Uest_breve - Y_breve).^2, 2)));
    exec_time(trial, :, noise, init_pose)         = t_end;

    % if debug mode go out of the loop
    if( displaybone )
        disp('Estimated');
        disp([t_all', eul_all]);
        disp('GT');
        disp(GT);
        disp('Error');
        disp(errors(trial, :, noise, init_pose));
        break;
    end
    
    % increase the index of the loop
    trial = trial+1;

% end trials
end

% save the trials if not debug mode
if (~displaybone)
    save( strcat(path_result, filesep, filename_result), ...
          'GTs', 'estimations', 'errors', 'rmse_measurements', 'rmse_trues', 'exec_time', 'description');
% if debug mode, go out of the loop
else
    break;
end

% end init_poses
end

% if debug mode go out of the loop
if( displaybone )
    break;
end

% end noise
end


