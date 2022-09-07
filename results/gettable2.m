% This script is to generate table with 6x36 format
% rows consist of 6 dof
% columns consist of 3*3*4; 3 statistical test, 3 initpose, 4 noise

clear; close all;
addpath(genpath('..\functions\display'));

%filename = 'backup/amode_old/goicptrials_bone';
filepath = 'backup/amode_new/tibia/trials2';
filename = 'ukfnormal_30_trials2';
load(strcat(filepath, filesep, filename,'.mat'));

initposes_sel = 4;
noises_sel    = [2, 3, 4, 5];

n_initposes  = length(description.init_poses(initposes_sel));
n_noises     = length(description.noises(noises_sel));
n_dofs       = 6;
n_dataincell = 3;

data = zeros(n_dofs, n_initposes*n_noises*n_dataincell);

for dof=1:n_dofs
    column_idx = 1;
    for init_pose=1:n_initposes   
        for noise=1:n_noises

            % data_idx = 1;
            % data_temp = boxplot(absolute_errors(:, dof, noise, init_pose));
            data_temp = boxplot(abs(errors( :, dof, ...
                                            noises_sel(noise), ...
                                            initposes_sel(init_pose)...
                                          )));

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

            dataincell = [round(median(1),3), round(upperadjacent(1),3), round(num_outliers)];
            for i=1:length(dataincell)
                
                if (dof<=3)
                    data(dof, column_idx) = dataincell(i);
                else
                    dof_reversed = 3 + abs(dof-(n_dofs+1));
                    data(dof_reversed, column_idx) = dataincell(i);                    
                end
                
                column_idx = column_idx+1;
            end
        end
    end
end