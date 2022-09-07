function [T, rmse_measurement] = fineregistration(moving, fixed, algorithm, path_to_function)
%FINEREGISTRATION Summary of this function goes here
%   Detailed explanation goes here

if (strcmp(algorithm, 'icp'))

    % ICP Registration
    % change the point structure to be suit to matlab icp built in function
    movingPC = pointCloud(moving);
    fixedPC  = pointCloud(fixed);
    % register with icp
    [tform, ~, icp_rmse] = pcregistericp( movingPC, ...
                                          fixedPC, ...
                                          'InlierRatio', 1, ...
                                          'Verbose', false, ...
                                          'MaxIteration', 50 );

    % change the T form, for me it is easier to work with [R, t; 0 1] than
    % [R, 0; t 1] which provided by matlab.
    T   = tform.T';
    % store the rmse
    rmse_measurement = icp_rmse;

elseif (strcmp(algorithm, 'cpd'))

    % CPD Registration
    % change the point structure to be suit to matlab icp built in function
    moving = pointCloud(moving*1000);
    fixed  = pcdownsample( pointCloud(fixed*1000), 'gridAverage', 2);
    % register with icp
    [tform, ~, cpd_rmse] = pcregistercpd( moving, ...
                                          fixed, ...
                                          'Transform', 'Rigid', ...
                                          'OutlierRatio', 0.9, ...
                                          'MaxIteration', 250, ...
                                          'Tolerance', 1e-8, ...
                                          'verbose', false);

    % change the T form, for me it is easier to work with [R, t; 0 1] than
    % [R, 0; t 1] which provided by matlab.
    T   = tform.T';
    T(1:3, 4) = T(1:3, 4)/1000;
    % store the rmse
    rmse_measurement = cpd_rmse;

elseif (strcmp(algorithm, 'ukf'))

    % UKF Registration
    [T, mean_dist] = ukf_isotropic_registration( moving, fixed, [], ...
                           'threshold', 0.5, ...
                           'iteration', 150, ...
                           'expectednoise', 1.25*var_yacute, ...
                           'sigmaxanneal', 0.98, ...
                           'sigmaxtrans', 1.2*max_t, ...
                           'sigmaxtheta', 1.2*max_theta, ...
                           'verbose', false, ...
                           'display', false);
    % store the rmse
    rmse_measurement = mean_dist;

elseif (strcmp(algorithm, 'goicp'))

    % GO-ICP Registration
    % normalize everything
    temp = [moving; fixed];
    scale = max(max(temp));
    temp = temp ./ scale;
    data = temp(1:size(moving, 2), :);
    model = temp(size(moving, 2)+1:end, :);
    % store data.txt
    fileID = fopen('data\temp\data.txt','w');
    fprintf(fileID,'%d\n', size(data, 1));
    fprintf(fileID,'%f %f %f\n', data');
    fclose(fileID);
    % store model.txt
    fileID = fopen('data\temp\model.txt','w');
    fprintf(fileID,'%d\n',  size(model, 1));
    fprintf(fileID,'%f %f %f\n', model');
    fclose(fileID);    
    % run GO-ICP
    goicp_exe  = "GoICP_vc2012";
    cmd = sprintf("%s %s %s %d %s %s", ...
                  strcat(path_to_function, filesep, "bin", filesep, goicp_exe), ...
                  "data\temp\model.txt", ...
                  "data\temp\data.txt", ...
                  size(moving,1), ...
                  strcat(path_to_function, filesep, "demo", filesep, "config.txt"), ...
                  "data\temp\output.txt");
    system(cmd);
    % open output file
    file = fopen('data\temp\output.txt', 'r');
    time = fscanf(file, '%f', 1);
    R    = fscanf(file, '%f', [3,3])';
    t    = fscanf(file, '%f', [3,1]) * scale;
    fclose(file);
    % reformat the T
    T = [R, t; 0 0 0 1];
    % no rmse reported, so give it NaN
    rmse_measurement = NaN;
else
    
    disp('There is no option for the specified registration algorithm');
    
end
    
    
end

