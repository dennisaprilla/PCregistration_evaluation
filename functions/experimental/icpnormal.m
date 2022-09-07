function [T_all, mean_dist] = icpnormal(moving, movingnormal, fixed, fixednormal, varargin)
%ICPNORMAL Summary of this function goes here
%   Detailed explanation goes here
p = inputParser;

addRequired( p, 'moving', ...
             @(x) validateattributes(x, {'double'}, {'ncols', 3}) );
addRequired( p, 'fixed', ...
             @(x) validateattributes(x, {'double'}, {'ncols', 3}) );
addRequired( p, 'movingnormal', ...
              @(x) validateattributes(x, {'double'}, {'ncols', 3}) );
addRequired( p, 'fixednormal', ...
              @(x) validateattributes(x, {'double'}, {'ncols', 3}) );
addOptional( p, 'movingdebug', zeros(3,1), ...
             @(x) validateattributes(x, {'double'}, {'ncols', 3}) );
         
addParameter( p, 'iteration', 1, ...
              @(x) isnumeric(x) );
addParameter( p, 'threshold', 1, ...
              @(x) isnumeric(x) );
addParameter( p, 'normalratio', 1, ...
              @(x) isnumeric(x) );
addParameter( p, 'ransacdistance', 1, ...
              @(x) isnumeric(x) );
addParameter( p, 'bestrmse', false, ...
              @(x) islogical(x) );
addParameter( p, 'verbose', true, ...
              @(x) islogical(x) );
addParameter( p, 'display', true, ...
              @(x) islogical(x) );          
parse(p, moving, movingnormal, fixed, fixednormal, varargin{:});

% renaming variable
U_breve     = p.Results.movingdebug;
U           = moving;
U_hat       = movingnormal;
Y_breve     = fixed;
Y_breve_hat = fixednormal;

% renaming parameter
iteration       = p.Results.iteration;
threshold      = p.Results.threshold;
normalratio    = p.Results.normalratio;
ransacdistance = p.Results.ransacdistance;

% plot for sanity check
if (p.Results.display)
    figure2 = figure('Name', 'ICP Normal Registration', 'Position', [0 0 350 780]);
    axes2 = axes('Parent', figure2);
    % plot the bone in original position
    plot3( axes2, ...
           U_breve(:,1), ...
           U_breve(:,2), ...
           U_breve(:,3), ...
           '.', 'Color', [0.7 0.7 0.7], ...
           'MarkerSize', 0.1, ...
           'Tag', 'plot2_Ubreve');
    grid on; axis equal; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    % plot the a-mode measurement
    plot3( axes2, ...
           U(:,1), ...
           U(:,2), ...
           U(:,3), ...
           'or', ...
           'Tag', 'plot2_U_noised');
    % plot the a-mode measurement normal
    quiver3(axes2, ...
            U(:,1),     U(:,2),     U(:,3), ...
            U_hat(:,1), U_hat(:,2), U_hat(:,3), 0.1, ...
            'Tag', 'plot2_Uhat_noised');
    % plot the target bone measurement
    plot3( axes2, ...
           Y_breve(:,1), ...
           Y_breve(:,2), ...
           Y_breve(:,3), ...
           '.g', 'MarkerSize', 0.1, ...
           'Tag', 'plot2_Ybreve');   
end

% initialization
U_breve_current     = U_breve;
U_current           = U;
U_hat_current       = U_hat;
Y_breve_hat_current = Y_breve_hat;
T_all               = eye(4);

% start iteration
for iter=1:iteration
    
    % construct the current point position and orientation
    moving    = [U_current, U_hat_current * normalratio];
    fixed     = [Y_breve, Y_breve_hat_current * normalratio];
    % search for correspondences using normal
    nearest_idx   = knnsearch(fixed, moving, 'Distance', 'euclidean');
    nearest_point = Y_breve(nearest_idx, :);

    % plot for sanity check
    if (p.Results.display)
        % plot the correspondences point
        plot3( axes2, ...
               nearest_point(:,1), ...
               nearest_point(:,2), ...
               nearest_point(:,3), ...
               'dr', 'MarkerFaceColor', 'm', ...
               'Tag', 'plot2_pointpair');
        % plot the line for each of correspondences point
        for j=1:size(U_current,1)
            temp_pointPair = [U_current(j,:); nearest_point(j,:)];
            plot3( axes2, temp_pointPair(:,1), temp_pointPair(:,2), temp_pointPair(:,3), ...
                   '-m', 'Tag', 'plot2_pointpair');
        end
        
        drawnow;
    end

    % estimate the transformation
    try
        tform = estimateGeometricTransform3D( U_current, ...
                                              nearest_point, ...
                                              'rigid', ...
                                              'MaxDistance', ransacdistance );
    catch
        warning('icpnormal encounter a problem: not enough inlier to estimate tform');
        T_all = eye(4);
        mean_dist = NaN;
        return;
    end
    
    % rearange the T (because i like this kind of form) and store it
    T_current = tform.T';
    T_all     = T_current * T_all;
    % extract the T and transform
    t_current = T_current(1:3, 4);
    R_current = T_current(1:3, 1:3);
    U_current     = (R_current * U_current' + t_current)';
    U_hat_current = (R_current * U_hat_current')';
    % transform Ubreve so we know what is happening, we dont use this for
    % transformation estimation, only for display
    U_breve_current      = (R_current * U_breve_current' + t_current)';

    % plot for sanity check
    if (p.Results.display)
        delete(findobj('Tag', 'plot2_pointpair'));
        delete(findobj('Tag', 'plot2_Ubreve'));
        delete(findobj('Tag', 'plot2_U_noised'));
        delete(findobj('Tag', 'plot2_Uhat_noised'));
        % plot the transformed bone
        plot3( axes2, ...
               U_breve_current(:,1), ...
               U_breve_current(:,2), ...
               U_breve_current(:,3), ...
               '.', 'Color', [0.7 0.7 0.7], ...
               'MarkerSize', 0.1, ...
               'Tag', 'plot2_Ubreve');
        % plot the transformed a-mode
        plot3( axes2, ...
               U_current(:,1), ...
               U_current(:,2), ...
               U_current(:,3), ...
               'or', ...
               'Tag', 'plot2_U_noised');
        % plot the transformed normal
        quiver3(axes2, ...
                U_current(:,1),     U_current(:,2),     U_current(:,3), ...
                U_hat_current(:,1), U_hat_current(:,2), U_hat_current(:,3), 0.1, ...
                'Tag', 'plot2_Uhat_noised');
            
        drawnow;
    end

    % calculate rmse
    rmse_current =  mean(sqrt(sum((U_current - nearest_point).^2)));
    
    if (p.Results.verbose)
        % print current iteration, point used, and mean distance
        fprintf('%d %.4f\n\n', iter, rmse_current);
        
        % print current estimated transformation
        t_all   = T_all(1:3, 4);
        R_all   = T_all(1:3, 1:3);
        eul_all = rad2deg(rotm2eul(R_all, 'ZYX'));
        disp([t_all', eul_all]);
    end
    
    % breaking point
    if (rmse_current<threshold)
        break;
    end

end

% return the mean dist
mean_dist = rmse_current;

end

