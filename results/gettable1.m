% This script is to generate table with 18x12 format
% rows consist of 3*6; 3 statistical test, 6 dof
% columns consist of 3*4; 3 initpose, 4 noise

clear; close all;
addpath(genpath('..\functions\display'));

filename = 'backup/amode_old/goicptrials_bone';
load(strcat(filename,'.mat'));

n_initposes  = length(description.init_poses);
n_noises     = length(description.noises);
n_dofs       = 6;
n_dataincell = 3;

data = zeros(n_dofs*n_dataincell, n_initposes*n_noises);
column_idx = 1;
for init_pose=1:n_initposes   
    for noise=1:n_noises
        
        data_idx = 1;
        for dof=1:n_dofs
            data_temp = boxplot(absolute_errors(:, dof, noise, init_pose));
            
            line1 = findobj(gcf,'tag','Upper Adjacent Value');
            line2 = findobj(gcf,'tag','Median');
            line3 = findobj(gcf,'tag','Outliers');
            
            upperadjacent = get(line1, 'YData');
            median = get(line2, 'YData');
            outliers = get(line3, 'YData');
            
            if (isnan(outliers))
                num_outliers = 0;
            else
                num_outliers = length(outliers);
            end
            
            % i need to do this if because the order for rz is flipped,
            % instead of (rx, ry, rz), it is (rz, ry, rx). if you put the
            % order nicely in the mat file, delete the else part of the if
            if(dof<=3)
                % 1:3, 4:6, 7:9, ...
                % data(data_idx:data_idx+(n_dataincell-1), column_idx) = [round(median(1),2); round(upperadjacent(1),2); round(num_outliers)];
                % data_idx = data_idx+n_dataincell;
                start_idx = ((dof-1)*3)+1;
                end_idx   = (start_idx-1) + n_dataincell;
                data(start_idx:end_idx, column_idx) = [round(median(1),2); round(upperadjacent(1),2); round(num_outliers)];
            else
                % do some crazy shit manipulation for indexing
                data_idx_reversed = 3 + abs(dof-(n_dofs+1));
                start_idx = ((data_idx_reversed-1)*3)+1;
                end_idx   = (start_idx-1) + n_dataincell;
                data(start_idx:end_idx, column_idx) = [round(median(1),2); round(upperadjacent(1),2); round(num_outliers)];          

            end
        end
        
        data_idx   = 0;
        column_idx = column_idx+1;
    end
end

