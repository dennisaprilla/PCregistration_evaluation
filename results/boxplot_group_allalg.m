% This script is used to create 2x3 plot, each plot is for each
% trasnformation. In each plot divided into number of noise, and each noise
% divided into number of algorithm.

clear; close all;
addpath(genpath('..\functions\display'));
addpath('..\functions\external\ini2struct');
plotconfig = ini2struct('plotconf.ini');

% specify source
sourcepath     = plotconfig.path.sourcepath;
% specify output folder
resultpath     = plotconfig.path.resultpath;
% specify the file
bone           = plotconfig.file.bone;
trialname      = plotconfig.file.trialname;

% if you want to compare all algorithm, use 'compare_alg'
% if you want to compare point numbers, use 'compare_point'
% if you want to compare point configuration, use 'compare_config'
display_config = plotconfig.display.display_mode;
% save picture?
save_picture   = strcmp(plotconfig.display.save_picture, 'true');
% limit error to visualized
ymax           = str2double(plotconfig.display.yaxis_max);
yticks         = (1:2:ymax);

% compare algorithm will show all 6 DoF
if(strcmp(display_config, 'compare_alg'))
	% specify source details
    sourcefullpath = strcat(sourcepath, filesep, bone, filesep, trialname);
    % specify source files
    filenames  = { sprintf('%s_%d_%s', 'icp', 15, trialname), ...
                   sprintf('%s_%d_%s', 'cpdmatlab', 15, trialname), ...
                   sprintf('%s_%d_%s', 'ukf', 15, trialname), ...
                   sprintf('%s_%d_%s', 'goicp', 15, trialname), ...
                   sprintf('%s_%d_%s', 'fricp', 15, trialname), ...
                   sprintf('%s_%d_%s', 'rsicp', 15, trialname), ...
                   sprintf('%s_%d_%s', 'icpnormal', 15, trialname), ...
                   sprintf('%s_%d_%s', 'ukfnormal', 15, trialname) };
    alg_names  = {'ICP', 'CPD', 'UKF', 'GOICP', 'FRICP', 'RSICP', 'ICP+norm', 'UKF+norm'};
    
    % specify output folder details
    outputcategory = 'algorithm_comparison';
    resultfullpath = strcat(resultpath, filesep, bone, filesep, outputcategory);
    % specify output files
    outputname     = sprintf('%s_%s_abserror', 'allalg', bone);
    
    
    
% compare point will show only tz and Rz
elseif(strcmp(display_config, 'compare_point'))
	% specify source details
    sourcefullpath = strcat(sourcepath, filesep, bone, filesep, trialname);
    % specify source files
    alg_used   = plotconfig.displaydetail.algorithm;
    filenames  = { sprintf('%s_%d_%s', alg_used, 10, trialname), ...
                   sprintf('%s_%d_%s', alg_used, 15, trialname), ...
                   sprintf('%s_%d_%s', alg_used, 20, trialname), ...
                   sprintf('%s_%d_%s', alg_used, 25, trialname), ...
                   sprintf('%s_%d_%s', alg_used, 30, trialname) };
    alg_names  = {'10', '15', '20', '25', '30'};    

    % specify output folder details
    outputcategory = 'sensitivity_pointnumber';
    resultfullpath = strcat(resultpath, filesep, bone, filesep, outputcategory);
    % specify output files
    modename       = 'point';
    displayname    = sprintf('all%s', alg_used);
    outputname     = { sprintf('%s_%s_%s_tz_abserror', displayname, bone, modename), ...
                       sprintf('%s_%s_%s_rz_abserror', displayname, bone, modename) };
                   
                   
                   
% compare config will show only tz and Rz
elseif(strcmp(display_config, 'compare_config'))
	% specify source details
    sourcefullpath = strcat(sourcepath, filesep, bone, filesep, trialname);
    % specify source files
    alg_used   = plotconfig.displaydetail.algorithm;
    filenames  = { sprintf('%s_%d_conf%d_%s', alg_used, 15, 1, trialname), ...
                   sprintf('%s_%d_conf%d_%s', alg_used, 15, 2, trialname), ...
                   sprintf('%s_%d_conf%d_%s', alg_used, 15, 3, trialname)};
    alg_names  = {'Config 1', 'Config 2', 'Config 3'};
   
    % specify output folder details
    outputcategory = 'sensitivity_pointconfig';
    resultfullpath = strcat(resultpath, filesep, bone, filesep, outputcategory);
    % specify output files
    modename       = 'config';
    displayname    = sprintf('all%s', alg_used);
    outputname     = { sprintf('%s_%s_%s_tz_abserror', displayname, bone, modename), ...
                       sprintf('%s_%s_%s_rz_abserror', displayname, bone, modename) };

                   
                   
% use this if you want to make a custom display
else
    % specify source details
    sourcefullpath = strcat(sourcepath, filesep, bone, filesep, trialname);
    % specify source files
    filenames  = { 'cpdmyronenko_15_trials0', 'cpdmyronenko_15_trials1', 'cpdmyronenko_15_trials2'};
    alg_names  = {'Config 1', 'Config 2', 'Config 3'};
    
    % specify output folder details
    outputcategory = 'sensitivity_pointconfig';
    resultfullpath = strcat(resultpath, filesep, bone, filesep, outputcategory);
    % specify output files
    outputname     = sprintf('%s_%s_abserror', 'allalg', bone);
    display_config = 'compare_alg';
end

% for visualization purpose
colorpalette = {'#57606f', '#5352ed', '#70a1ff', '#18dcff', '#2ed573', '#ffa502', '#e67e22', '#ff4757'};

%% Preparing Data

% storing some variable
total_algorithms = length(filenames);
total_dof        = 6;

% rearrange data, the requirement for boxplotGroup, please refer
% https://www.mathworks.com/matlabcentral/answers/331381-how-do-you-create-a-grouped-boxplot-with-categorical-variables-on-the-x-axis#answer_418952
data = {};
for filename_idx=1:total_algorithms
    load(strcat(sourcefullpath, filesep, filenames{filename_idx},'.mat'));
    
    % renaming variables
    init_poses       = description.init_poses;
    total_poses      = length(init_poses);
    init_poses_sel   = str2double(split(plotconfig.displaydetail.initposes_idxsel, ','))';
    noises           = description.noises;
    total_noises     = length(noises);
    noises_sel       = str2double(split(plotconfig.displaydetail.noises_idxsel, ','))';
    total_noises_sel = length(noises_sel);
    
    for dof_idx=1:total_dof
        data{filename_idx, dof_idx} = reshape( abs( errors(:, dof_idx, noises_sel, init_poses_sel )), [], total_noises_sel);
    end
    
end
data = data';

%% Visualization

if (strcmp(display_config, 'compare_alg'))

    % we use subaxis function to control more for the spacing for the subplot
    % https://www.mathworks.com/matlabcentral/fileexchange/3696-subaxis-subplot
    fig1   = figure('Name', 'Error distribution', 'Position', [0 0 1200 700]);
    titles = {'Error distribution t_x (mm)', 'Error distribution t_y (mm)', 'Error distribution t_z (mm)', ...
              'Error distribution R_x (deg)', 'Error distribution R_y (deg)', 'Error distribution R_z (deg)'};
    for dof_idx=1:total_dof

        % prepare the subaxis
        subaxis( 2,3, dof_idx, ...
                 'SpacingVertical',0.13, 'SpacingHorizontal', 0.05, ...
                 'MarginLeft', 0, 'MarginRight', 0, 'MarginTop', 0.05);
        hold on;

        % get the dof
        if(dof_idx <=3)
            data_temp = data(dof_idx, :);
        else
            data_temp = data(abs(dof_idx-(total_dof+1))+3, :);
        end

        % draw the box plot
        h = boxplotGroup( data_temp, ...
                          'PrimaryLabels', alg_names, ...
                          'SecondaryLabels', strcat('Noise', {' '}, arrayfun(@num2str, noises(noises_sel), 'UniformOutput', 0)), ...
                          'GroupLabelType', 'Vertical', ...
                          'interGroupSpace', 1, ...
                          'BoxStyle', 'filled', ...
                          'Symbol', '.', ...
                          'MedianStyle', 'target');
        xtickangle(60);

        % coloring the box plot
        start_boxelement = (length(h.axis.Children) - total_algorithms)+1;
        end_boxelement   = length(h.axis.Children);
        colorpallete_idx = length(colorpalette);
        for element = end_boxelement:-1:start_boxelement
            set(h.axis.Children(element).Children,'Color', colorpalette{colorpallete_idx});
            colorpallete_idx = colorpallete_idx-1;
        end

        % coloring the dots (median and outliers)
        median_obj = findobj(gcf, 'Tag', 'MedianOuter');
        set(median_obj, 'MarkerSize', 6);
        median_obj = findobj(gcf, 'Tag', 'MedianInner');
        set(median_obj, 'MarkerEdgeColor', 'r');
        outlier_obj = findobj(gcf, 'Tag', 'Outliers');
        set(outlier_obj, 'MarkerEdgeColor', [0.8, 0.8, 0.8]); 
        grid on;

        % limit the y_axis
        ylim([0, ymax]);
        set(gca,'YTick',(1:2:ymax));

        % set the title
        title(titles{dof_idx});
    end
    
    if (save_picture)
        % save the picture
        % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
        set(fig1,'Units','Inches');
        pos = get(fig1,'Position');
        set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
        print(fig1, strcat(resultfullpath, filesep, outputname), '-dpdf','-r0');
        saveas(fig1, strcat(resultfullpath, filesep, outputname), 'png');
    end

else
    
    titles        = {'t_z absolute error (mm)', 'R_z absolute error  (deg)'};
    start_dofidx  = 3;
    end_dofidx    = 4;
    sigstar_isall = false; % true  = (1,2), (1,3), (1,4), (2,3), (2,4), (3,4), ...
                           % false = (1,2), (2,3), (3,4), ... 
    for dof_idx=start_dofidx:end_dofidx
    
        % we use subaxis function to control more for the spacing for the subplot
        % https://www.mathworks.com/matlabcentral/fileexchange/3696-subaxis-subplot
        fig1 = figure('Name', 'Error distribution', 'Position', [0 0 500 200]);
        subaxis( 1,1, 1, ...
                 'SpacingVertical',0.15, 'SpacingHorizontal', 0.05, ...
                 'MarginLeft', 0.05, 'MarginRight', 0.01, 'MarginTop', 0.025);
        % get the data
        data_temp = data(dof_idx, :);
        % draw the box plot
        h = boxplotGroup( data_temp, ...
                          'PrimaryLabels', alg_names, ...
                          'SecondaryLabels', strcat('Noise', {' '}, arrayfun(@num2str, noises(noises_sel), 'UniformOutput', 0)), ...
                          'GroupLabelType', 'Vertical', ...
                          'interGroupSpace', 1, ...
                          'BoxStyle', 'filled', ...
                          'Symbol', '.', ...
                          'MedianStyle', 'target');
        xtickangle(60);

        % limit the y_axis -------------------------------------------------------------
        ylim([0, ymax]);
        set(gca,'YTick',yticks);
        
        % draw the significance star ----------------------------------------------------
        % i will put the significance star on the ymax only
        outlier_obj = findobj(gcf, 'Tag', 'Outliers');
        backup_outlier_obj = outlier_obj;
        for i=1:length(outlier_obj)
            del_idx = outlier_obj(i).YData>(ymax);
            outlier_obj(i).YData(del_idx) = [];
            outlier_obj(i).XData(del_idx) = [];
        end        
        
        % rearrange data (data required by boxplotgroup above is so confusing)
        data_temp_modified = {};
        for j=1:size(data_temp{1},2)
            data_temp_array = [];
            for i=1:length(data_temp)
                data_temp_array = [data_temp_array, data_temp{i}(:,j)];               
            end
            data_temp_modified{end+1} = data_temp_array;
        end
            
        % now we do our bussiness here
        n_boxplot_group = length(data_temp_modified);
        sigstar_group = {};
        sigstar_values = [];
        for current_boxplot_group=1:n_boxplot_group
            current_data_temp = data_temp_modified{current_boxplot_group};
            n_data_temp = size(current_data_temp,2);
            
            % because boxplot group treat a gap between group as one of the
            % boxplot, we need to put an offset so that our sigstar will
            % not displayed in the middle of boxplot group
            sigstar_offset = current_boxplot_group-1;
            
            % if want the sigstar displayed as: (1,2), (1,3), (1,4), (2,3), ...
            % this block program below will do
            if (sigstar_isall)
 
                % i want to make a loop to be (1,2), (1,3), (1,4), (2,3), ...
                for i=1:n_data_temp
                    for j=i+1:n_data_temp

                        % calculate the significance
                        p = signrank(current_data_temp(:,i),current_data_temp(:,j));
                        % if it is significance, store the group
                        if(p<0.005)
                            % I need to play with the index here. There are 
                            % groups of boxplot. The individual indexing of the
                            % boxplot is continously incremented from 1 till n. 
                            % So i need adjust from grouped index to overall index.
                            current_i = (n_data_temp * (current_boxplot_group-1)) + i + sigstar_offset;
                            current_j = (n_data_temp * (current_boxplot_group-1)) + j + sigstar_offset;
                            sigstar_group{end+1} = [current_i, current_j];

                            % Add p values so it can be displayed by the
                            % function. it is also tricky since it only provide
                            % [0.05 (*), 0.01(**), 0.001(***)], there is no
                            % option to change the stars, so that we force
                            % change it by playing with "stats" parameter.
                            if(p<0.001)
                                % here i put 0.01 because i want to have **
                                sigstar_values = [sigstar_values, 0.01];
                            else
                                % here i put 0.05 because i want to have *
                                sigstar_values = [sigstar_values, 0.05];
                            end
                        end
                    end
                end
                
            % if want the sigstar displayed as: (1,2), (2,3), (3,4) ...
            % this block program below will do
            else
                
                % i want to make loop to be (1,2), (2,3), (3,4)
                for i=1:(n_data_temp-1)
                    group1 = i;
                    group2 = i+1;

                    % calculate the significance
                    p = signrank(current_data_temp(:,group1),current_data_temp(:,group2));
                    % if it is significance, store the group
                    if(p<0.05)
                        % I need to play with the index here. There are 
                        % groups of boxplot. The individual indexing of the
                        % boxplot is continously incremented from 1 till n.
                        % So i need adjust from grouped index to overall index.
                        current_group1 = (n_data_temp * (current_boxplot_group-1)) + group1 + sigstar_offset;
                        current_group2 = (n_data_temp * (current_boxplot_group-1)) + group2 + sigstar_offset;
                        sigstar_group{end+1} = [current_group1, current_group2];

                        % Add p values so it can be displayed by the
                        % function. it is also tricky since it only provide
                        % [0.05 (*), 0.01(**), 0.001(***)], there is no
                        % option to change the stars, so that we force
                        % change it by playing with "stats" parameter.
                        if(p<0.01)
                            % here i put 0.01 because i want to have **
                            sigstar_values = [sigstar_values, 0.01];
                        else
                            % here i put 0.05 because i want to have *
                            sigstar_values = [sigstar_values, 0.05];
                        end
                    end
                end
                
            end

           
        end
        sigstar(sigstar_group, sigstar_values);
        
        % coloring the box plot -------------------------------------------------------
        start_boxelement = (length(h.axis.Children) - total_algorithms)+1;
        end_boxelement   = length(h.axis.Children);
        colorpallete_idx = length(colorpalette);
        % for element = start_boxelement : end_boxelement
        for element = end_boxelement:-1:start_boxelement
            % color_pallete_idx = (element - start_boxelement) + 1;
            set(h.axis.Children(element).Children,'Color', colorpalette{colorpallete_idx});
            colorpallete_idx = colorpallete_idx-1;
        end

        % coloring the dots (median and outliers) --------------------------------------
        median_obj = findobj(gcf, 'Tag', 'MedianOuter');
        set(median_obj, 'MarkerSize', 6);
        median_obj = findobj(gcf, 'Tag', 'MedianInner');
        set(median_obj, 'MarkerEdgeColor', 'r');
        outlier_obj = findobj(gcf, 'Tag', 'Outliers');
        set(outlier_obj, 'MarkerEdgeColor', [0.3412, 0.3961, 0.4549]);
        whisker_obj = findobj(gcf, 'Tag', 'Whisker');
        grid on; hold on;
        
        % line plot the median ---------------------------------------------------------
        total_noises  = total_noises_sel;
        total_configs = size(data_temp, 2);
        median_lines  = zeros(total_noises, 2, total_configs);
        upperwhisker_lines  = zeros(total_noises, 2, total_configs);
        lowerwhisker_lines  = zeros(total_noises, 2, total_configs);
        boxplot_median_idx = 1;
        for i=1:total_configs
            for j=1:total_noises
                median_lines(j,:,i)        = [median_obj(boxplot_median_idx).XData, median_obj(boxplot_median_idx).YData];
                lowerwhisker_lines(j,:,i)  = [whisker_obj(boxplot_median_idx).XData(1), whisker_obj(boxplot_median_idx).YData(1)];
                upperwhisker_lines(j,:,i)  = [whisker_obj(boxplot_median_idx).XData(2), whisker_obj(boxplot_median_idx).YData(2)];
                boxplot_median_idx = boxplot_median_idx+1;
            end
        end
        for i=1:size(median_lines, 1)
            plot( squeeze(median_lines(i,1,:)), squeeze(median_lines(i,2,:)), '-r' );
            
            x  = squeeze(lowerwhisker_lines(i,1,:));
            y1 = squeeze(lowerwhisker_lines(i,2,:));
            y2 = squeeze(upperwhisker_lines(i,2,:));
            where = x>=x(end) & x<=x(1);
            opts  = {'FaceColor', '#a4b0be', 'FaceAlpha', .2, 'LineStyle', 'none',};
            fill_between( x, y1, y2, where, opts{:} );
        end

        % set the title ----------------------------------------------------------------
        % prepare the subaxis
        title_idx = (dof_idx-start_dofidx)+1;
        ylabel(titles{title_idx});
        
        if (save_picture)
            % save the picture
            % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
            set(fig1,'Units','Inches');
            pos = get(fig1,'Position');
            set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
            print(fig1, strcat(resultfullpath, filesep, outputname{title_idx}), '-dpdf','-r0');
            saveas(fig1, strcat(resultfullpath, filesep, outputname{title_idx}), 'png');
        end
    end
    
end
