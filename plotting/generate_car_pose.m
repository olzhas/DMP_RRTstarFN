function generate_car_pose(iter, path_prefix)
close all;
%%
% figure;
% grid on;
% hold on;
% box on;
% axis square;
% xlim([0 2]);
% ylim([0 2]);

car_width = 80;
car_height =50;
car_init = [-40 -25; -40 25; 40 25; 40 -25];


path_filename = [path_prefix 'dubins-results-interp' num2str(iter)  '.txt'];

%% file read

fPath = fopen(path_filename);

if (fPath < 0)
    disp('File does not exist or another error');
    return
end

cell = textscan(fPath, '%f %f %f');
P = cell2mat(cell);

fclose(fPath);

%% file write initialization routine
temp = zeros(5,2);
prefix = ['car-temporal-' num2str(iter)];
mkdir(prefix);

%% plot
if exist('P', 'var')
    %plot(P(:,1), P(:,2), '.-r', 'LineWidth', 2);
    
    for ind = 1:size(P,1)
        path_output = [ prefix '/dubins-results-car-pose' num2str(ind)  '.dat'];
        fOutput = fopen(path_output, 'w');
        
        rot = [cos(P(ind,3)) -sin(P(ind,3)); sin(P(ind,3)) cos(P(ind,3))];
        car_box = (rot * car_init')';
        car_box(:,1) = car_box(:,1) + P(ind,1);
        car_box(:,2) = car_box(:,2) + P(ind,2);   
        temp(1:4, :) = car_box;
        temp(5, :) = car_box(1,:);
        %fill(temp(:,1), temp(:,2), 'm');
        
        for vert = 1:(size(temp,1)-1)
            fprintf(fOutput, '%f %f %f %f\n', temp(vert,1), temp(vert,2), temp(vert+1, 1), temp(vert+1,2));
        end
        
        fclose(fOutput);
    end
end

end
