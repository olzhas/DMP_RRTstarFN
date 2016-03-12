clear all;

system('rm -rf *png');

mkdir('results');

% files = dir('dubins-edges*.dat');
files = 1:239;

mkdir('temp');

skip_output_mp4 = false;

if (skip_output_mp4 == false)
    
    for ind = 0:numel(files)
        
        res=['dubins-results-interp' num2str(ind) '.txt'];
        vert=['dubins-vertices' num2str(ind) '.dat'];
        edge=['dubins-edges' num2str(ind) '.dat'];
        fout=['output' num2str(ind) '.png'];
        
        solution = '';
        
        fsol = fopen(['dubins-results-is_solution' num2str(ind) '.txt' ]);
        if fsol > 0
            if (fscanf(fsol, '%d') == 1)
                solution = ['filenameResults=''' res ''' '];
                %disp 'Solution'
            end
            fclose(fsol);
        end
        
        cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameOutput=''' fout ''';'...
            'circle=1; ' solution '" tree.gnu'];
        system(cmd);
        
        if mod(ind,10) == 0
            disp (['Done ' num2str(ind)]);
        end
    end
    
    system('/usr/local/bin/avconv -framerate 25 -f image2 -i output%d.png -c:v h264 -crf 1 results/output.mp4 -y -loglevel panic');
    
    disp('Done: output.mp4');
    
end
%%

current_iter = 239;
generate_car_pose(current_iter,'');

files = dir(['car-temporal-' num2str(current_iter) '/*.dat']);
res   = ['dubins-results-interp' num2str(current_iter) '.txt'];
vert  = ['dubins-vertices' num2str(current_iter) '.dat'];
edge  = ['dubins-edges' num2str(current_iter) '.dat'];

disp(['car pos generation for snapshot ' num2str(current_iter) 'Done']);

%%
dynamic_iter = 35;
%system('rm -rf static_car*png');
for ind = 1:dynamic_iter
    fname = ['static_car' num2str(ind) '.png'];
    input_file = ['car-temporal-' num2str(239) '/dubins-results-car-pose' num2str(ind) '.dat'];
    
    solution = ['filenameResults=''' res ''' '];
    cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameOutput=''' fname ''';'...
        'circle=1; '...
        'car_pose_dat=''' input_file ''';' ...
        solution '" tree.gnu'];
    
    system(cmd);
end


system('/usr/local/bin/avconv -framerate 25 -f image2 -i static_car%d.png -c:v h264 -crf 1 results/car.mp4 -y -loglevel panic');

disp('Done: car.mp4');

%%
res = 'dubins-results-interp239.txt';
vert = 'dubins-vertices239.dat';
edge = 'dubins-edges239.dat';

input_file = ['car-temporal-' num2str(current_iter) '/dubins-results-car-pose' num2str(dynamic_iter) '.dat'];

fout = 'output-move-0.png';
cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert ''';' ...
    'filenameResults=''' res '''; filenameOutput=''' fout '''; circle=3; ' ...
    'car_pose_dat=''' input_file ''';" tree.gnu'];
system(cmd);

% fout = 'output-move-4.png';
% cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert ''';' ...
%     'filenameResults=''' res '''; filenameOutput=''' fout '''; circle=2; ' ...
%     'car_pose_dat=''' input_file ''';" tree.gnu'];
% system(cmd);

res = 'dubins-results-interp239.txt';
vert = 'dubins-vertices239.dat';
vertHighlight = 'dynamic_removal/dubins-vertices800.dat';

edge = 'dubins-edges239.dat';
edgeHighlight= 'dynamic_removal/dubins-edges800.dat';
foutOne = 'output-move-1.png';
foutTwo = 'output-move-2.png';

cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; ' ...
    'filenameResults=''' res '''; filenameOutput=''' foutOne '''; circle=2; ' ...
    'edgeHighlight=''' edgeHighlight '''; vertHighlight=''' vertHighlight '''; highlightStyle=1; ' ...
    'car_pose_dat=''' input_file ''';" tree.gnu'];
system(cmd);

cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; ' ...
    'filenameResults=''' res '''; filenameOutput=''' foutTwo '''; circle=2; ' ...
    'edgeHighlight=''' edgeHighlight '''; vertHighlight=''' vertHighlight '''; highlightStyle=2; ' ...
    'car_pose_dat=''' input_file ''';" tree.gnu'];
system(cmd);

%%
res = 'dubins-results-interp239.txt';
vert = 'dynamic_removal/dubins-vertices800.dat';
edge = 'dynamic_removal/dubins-edges800.dat';
fout = 'output-move-3.png';

cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert ''';' ...
    'filenameResults=''' res '''; filenameOutput=''' fout '''; circle=2; ' ...
    'car_pose_dat=''' input_file ''';" tree.gnu'];
system(cmd);

system('/usr/local/bin/avconv -framerate 1 -f image2 -i output-move-%d.png -c:v h264 -crf 1 results/recover.mp4 -y -loglevel panic');

disp('Done: recover.mp4');

%%

generate_car_pose(241,'recovered_path/');
n = 800;

f = fopen(['recovered_path/dubins-results-interp' num2str(241) '.txt']);
solution_path='';
if (f > 0)
    solution_path = textscan(f, '%f %f %f');
    fclose(f);
end

edge = 'recovered_path/dubins-edges241.dat';
vert = 'recovered_path/dubins-vertices241.dat';

mkdir('replan-png');

for ind = 1:numel(solution_path{1})
    fname = ['replan-png/replan' num2str(ind) '.png'];
    input_file = ['car-temporal-241/dubins-results-car-pose' num2str(ind) '.dat'];
    res='recovered_path/dubins-results-interp241.txt';
    solution = ['filenameResults=''' res ''' '];
    cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameOutput=''' fname ''';'...
        'circle=2; '...
        'car_pose_dat=''' input_file ''';' ...
        solution '" tree.gnu'];
    
    system(cmd);
end

system('/usr/local/bin/avconv -framerate 25 -f image2 -i replan-png/replan%d.png -c:v h264 -crf 1 results/replan.mp4 -y -loglevel panic');

disp('Done: replan.mp4');

%%
% change this to make it work
switch_snapshot = 7;

prefix = 'path-color/';

files = dir([prefix '*dat']);
n = 800;
input_file = ['car-temporal-239/dubins-results-car-pose' num2str(dynamic_iter) '.dat'];

for ind = 1:switch_snapshot
    
    res=[prefix 'dubins-colored-results-' num2str(ind) '.dat'];
    vert=['dynamic_removal/dubins-vertices' num2str(n) '.dat'];
    edge=['dynamic_removal/dubins-edges' num2str(n) '.dat'];
    fout=[prefix 'colored' num2str(ind) '.png'];
    
    cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameResults=''' res ''';'...
        'filenameOutput=''' fout '''; circle=2; obstacle=1; car_pose_dat=''' input_file ''';" tree.gnu'];
    
    system(cmd);
end

n = 801;
for ind = switch_snapshot:numel(files)
    
    res=[prefix 'dubins-colored-results-' num2str(ind) '.dat'];
    vert=['dynamic_removal/dubins-vertices' num2str(n) '.dat'];
    edge=['dynamic_removal/dubins-edges' num2str(n) '.dat'];
    fout=[prefix 'colored' num2str(ind) '.png'];
    
    cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameResults=''' res ''';'...
        'filenameOutput=''' fout '''; circle=2; obstacle=1; car_pose_dat=''' input_file ''';" tree.gnu'];
    
    system(cmd);
    
end

system('/usr/local/bin/avconv -framerate 2 -f image2 -i path-color/colored%d.png -c:v h264 -crf 1 results/colored.mp4 -y -loglevel panic');

display('Done: colored.mp4');

%%
input_file = ['car-temporal-239/dubins-results-car-pose' num2str(dynamic_iter) '.dat'];
fout = 'results/orphaned1.png'
cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert ''';'... % filenameResults=''' res ''';'...
    'filenameOutput=''' fout '''; circle=2; car_pose_dat=''' input_file '''; dynamic=1" tree.gnu'];

system(cmd);

ind = 241;

res=['recovered_path/dubins-results-interp' num2str(ind) '.txt'];
vert=['recovered_path/dubins-vertices' num2str(ind) '.dat'];
edge=['recovered_path/dubins-edges' num2str(ind) '.dat'];
input_file = ['car-temporal-239/dubins-results-car-pose' num2str(dynamic_iter) '.dat'];
fout = 'results/orphaned2.png';

cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert ''';'... % filenameResults=''' res ''';'...
    'filenameOutput=''' fout '''; circle=2; car_pose_dat=''' input_file '''; dynamic=1" tree.gnu'];

system(cmd);

fout = 'results/orphaned3.png';
cmd = ['/usr/local/bin/gnuplot -e "filenameEdges=''' edge '''; filenameVertices=''' vert '''; filenameResults=''' res ''';'...
    'filenameOutput=''' fout '''; circle=2; car_pose_dat=''' input_file '''; dynamic=1" tree.gnu'];

system(cmd);
