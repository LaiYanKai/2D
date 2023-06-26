%%
clear all; close all; clc

algo = "VG2B";
scen = 0;

% map_pair = ["dao", "arena"];
% map_pair = ["dao", "hrt201n"];
% map_pair = ["sc1", "Aftershock"];
% map_pair = ["sc1", "Aurora"];
% map_pair = ["sc1", "ArcticStation"];
% map_pair = ["da2", "lt_0_lowtown_a3_n_c"];
% map_pair = ["room", "16room_000"];
map_pair = ["random", "random512-10-1_scale2"];
% get paths
script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
name = map_pair(2);
data_dir = fullfile(script_dir, "..", "..", "data", map_pair(1));
results_dir = fullfile(script_dir, "..", "..", "results", map_pair(1));
addpath(script_dir);

% display map
[M, I, C] = parse_maps(data_dir, name, true);

% get scenario
S = get_scenario(results_dir, name, algo, scen);

if ~isempty(S.path)
    % plot path
    hold on
    plot(S.path(:,1), S.path(:,2), '-', 'Color', [0.5,0.5,0.5]);
    num_points = height(S.path);
    for i = 1:num_points
        plot(S.path(i, 1), S.path(i, 2), '.', 'MarkerSize', 10, ...
            'Color', [0, (num_points-i)/num_points, i/num_points])
    end
    hold off

    % find limits to zoom into path
    zt = 10;
    xmax = max(S.path(:,1)) + zt;
    xmin = min(S.path(:,1)) - zt;
    ymax = max(S.path(:,2)) + zt;
    ymin = min(S.path(:,2)) - zt;
    dx = xmax - xmin;
    dy = ymax - ymin;
    rx = 16;
    ry = 9;
    if dx/ dy > rx / ry
        cy = dx / rx * ry;
        ymin = ymin - cy/2;
        ymax = ymax + cy/2;
    else
        cx = dy / ry * rx;
        xmin = xmin - cx/2;
        xmax = xmax + cx/2;
    end
    xlim([xmin, xmax])
    ylim([ymin, ymax])
end

%% plot another path in the same map and scenario
algo2 = "ANYA2B";
S2 = get_scenario(results_dir, name, algo2, scen);
if ~isempty(S2.path)
    hold on
    plot(S2.path(:, 1), S2.path(:, 2), 'r.-');
    hold off
end
%% plot a manual path in the same map
path = [
      489,208; 493,192; 496,185; 504,177; 504,160; 500,145; 500,144; 502,128; 507,125
    ];
fprintf("PathCost %.10f\n", sum(vecnorm(diff(path), 2, 2)));
hold on
plot(path(:,1), path(:,2), 'b.-')
hold off