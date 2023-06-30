%% Import and collate results, and save them
% also gther map characteristics
clear all; clc; close all

algs = ["R2E", "R2"]; % make sure the first alg returns the optimal path, and the optimal path does not contain more than 3 consecutive colinear points anywhere
expt_nums = 0:9;
map_pairs = [
    "dao", "arena_scale2";
        "bg512", "AR0709SR_scale2";
        "bg512", "AR0504SR_scale2";
        "bg512", "AR0014SR_scale2";
        "bg512", "AR0304SR_scale2";
        "bg512", "AR0702SR_scale2";
        "bg512", "AR0205SR_scale2";
        "bg512", "AR0602SR_scale2";
        "bg512", "AR0603SR_scale2";
        "street", "Denver_2_1024_scale2";
        "street", "NewYork_0_1024_scale2";
        "street", "Shanghai_2_1024_scale2";
        "street", "Shanghai_0_1024_scale2";
        "street", "Sydney_1_1024_scale2";
        "da2", "ht_mansion2b_scale2";
        "da2", "ht_0_hightown_scale2";
        "dao", "hrt201n_scale2";
        "random", "random512-10-1_scale2";
        "room", "32room_000_scale2";
        "room", "16room_000_scale2";
    ];


script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
cd(fullfile(script_dir, "..", ".."))

R = zeros(height(map_pairs), 1); % correlation coefficient of costs and points
N = zeros(height(map_pairs), 3); % num_cells, num_free_cells, num_corners
IM = cell(height(map_pairs), 1);
T_points = cell(height(map_pairs), 1);
T_costs = cell(height(map_pairs), 1);
T_nsecs = cell(height(map_pairs), numel(algs)); % map (row) x algo (col)
for m = 1:height(map_pairs)
    results_dir = fullfile("results", map_pairs(m, 1));
    name = map_pairs(m,2);
    specified = false;
    for a = 1:numel(algs)
        algo = algs(a);
        % calculate average run times across all expts for each scen,
        % and specify the optimal path num turning points and costs
        % assuming all paths found are identical across expts
        avg_nsecs = [];
        for expt_num = expt_nums
            t = get_scenarios(results_dir, name, algo, expt_num);
            avg_nsecs = [avg_nsecs, t.nsec]; % gather all avg_nsecs
            if ~specified % assumes all algs return the same shortest paths
                T_points{m} = t.points;
                T_costs{m} = t.cost;
                R(m) = corr(t.points, t.cost);

                % open maps
                [M, I, C] = parse_maps(fullfile("data", map_pairs(m, 1)), name, false);
                N(m, 1) = numel(M.mp);
                N(m, 2) = sum(M.mp == 0);
                N(m, 3) = height(C) - 4;
                IM{m} = I;

                specified = true;
            end
        end
        % get average nsecs
        T_nsecs{m, a} = mean(avg_nsecs, 2);
    end
end
row_names = extractBefore(map_pairs(:, 1) + "/" + map_pairs(:, 2), "_scale2");

T_nsecs = cell2table(T_nsecs, "RowNames", row_names, "VariableNames", algs);
T_costs = cell2table(T_costs, "RowNames", row_names, "VariableNames", "costs");
T_points = cell2table(T_points, "RowNames", row_names, "VariableNames", "points");
R = array2table(R, "RowNames", row_names, "VariableNames", "Corr");
N = array2table(N, "RowNames", row_names, "VariableNames", ["Cells", "FreeCells", "Corners"]);
IM = cell2table(IM, "RowNames", row_names, "VariableNames", "Image");

save(fullfile(script_dir, "results.mat"), "T_nsecs", "T_costs", "T_points", "R", "N", "IM");

%% import ANYA files
t_nsecs = cell(height(map_pairs), 1);
for m = 1:height(map_pairs)
    avg_nsecs = [];
    for i = 0:0
        t = readtable(fullfile("results", map_pairs(m, 1), map_pairs(m, 2) + ".ANYA." + num2str(i) + ".results"), "FileType", "text");
        avg_nsecs = [avg_nsecs, t.runt_micro(:) * 1000];
    end
    t_nsecs{m} = mean(avg_nsecs, 2);
    fprintf("%s Number of ANYA.realcost discrepancies: %d\n", map_pairs(m, 2), sum(abs(t.realcost - T_costs.costs{m}) > 1e-8)); % compare costs
    fprintf("%s Number of ANYA.gridcost discrepancies: %d\n", map_pairs(m, 2), sum(abs(t.gridcost - T_costs.costs{m}) > 1e-8));
end
T_nsecs.("ANYA") = t_nsecs;
%% import RSP files
RSP_nsecs = cell(height(map_pairs), 1);
for m = 1:height(map_pairs)
    name = extractBefore(map_pairs(m, 2), "_scale2");
    avg_nsecs = [];
    for i = (expt_nums + 1) % started from 1
        t = readtable(fullfile("..", "rayscan", "results", map_pairs(m,1), name + "_" + num2str(i) + ".txt"), "FileType", "text");
        avg_nsecs = [avg_nsecs, t.searchNs];
    end
    RSP_nsecs{m} = mean(avg_nsecs, 2);
    disp("Tabulated Rsp " + name);
end

T_nsecs.("RSP") = RSP_nsecs;
save(fullfile(script_dir, "results.mat"), "T_nsecs", "T_costs", "T_points", "R", "N", "IM");
%% Get average runtimes and speedups per points
clear all; clc; close all

script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
load(fullfile(script_dir, "results.mat"));

algs = T_nsecs.Properties.VariableNames;
map_pairs = T_nsecs.Properties.RowNames;

% plot data table for latex
T = T_nsecs;
SU = cell(height(T), 4);
for m = 1:height(T)
    % gather averrage speed ups for paths with same number of turn. pts.
    points = T_points.points{m};
    unique_points = unique(points);
    SU_R2E_ANYA = zeros(numel(unique_points), 1);
    SU_R2_ANYA = zeros(numel(unique_points), 1);
    SU_R2E_RSP = zeros(numel(unique_points), 1);
    SU_R2_RSP = zeros(numel(unique_points), 1);
    for i = 1:numel(unique_points)
        idx = (points == unique_points(i));
        nsecs_R2 = T_nsecs.R2{m};
        nsecs_R2E = T_nsecs.R2E{m};
        nsecs_ANYA = T_nsecs.ANYA{m};
        nsecs_RSP = T_nsecs.RSP{m};
        SU_R2E_ANYA(i) = mean(nsecs_ANYA(idx) ./ nsecs_R2E(idx));
        SU_R2_ANYA(i) = mean(nsecs_ANYA(idx) ./ nsecs_R2(idx));
        SU_R2E_RSP(i) = mean(nsecs_RSP(idx) ./ nsecs_R2E(idx));
        SU_R2_RSP(i) = mean(nsecs_RSP(idx) ./ nsecs_R2(idx));
    end
    SU{m, 1} = unique_points;
    SU{m, 2} = SU_R2E_ANYA;
    SU{m, 3} = SU_R2_ANYA;
    SU{m, 4} = SU_R2E_RSP;
    SU{m, 5} = SU_R2_RSP;

    % collect average runtimes for each algo
    for a = 1:width(T)
        data = T{m ,a};
        T{m, a} = {mean(data{:})};
    end

    % calculate map characteristics
end
SU = cell2table(SU, "RowNames", T_nsecs.Properties.RowNames, ...
    "VariableNames", ["unique_points", "R2E_ANYA", "R2_ANYA", "R2E_RSP", "R2_RSP"]);
save(fullfile(script_dir, "results.mat"), "T", "T_nsecs", "T_costs", "T_points", "SU", "R", "N", "IM");
%% plot
clear all; close all; clc
script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
load(fullfile(script_dir, "results.mat"));

map_pairs = [
%         "dao", "arena",
    "random", "random512-10-1",
    "room", "32room_000";
    "da2", "ht_mansion2b";
    "bg512", "AR0014SR";
    "street", "NewYork_0_1024";
    "street", "Shanghai_2_1024"
    ];
figure (1)
set(gcf, 'Position',  [100, 100, 1115, 140*numel(map_pairs)]);
TL = tiledlayout(height(map_pairs), 8,'TileSpacing','Compact','Padding','None');

for m = 1:height(map_pairs)
    row_name = map_pairs(m, 1) + "/" + map_pairs(m, 2);
    map_name = map_pairs(m, 2);

    nexttile([1, 2])
    I = IM.Image{row_name};
    ih = imagesc(I, "XData", 0.5, "YData", 0.5);
    ylabel(map_name, 'Interpreter','none')
    h1 = text(-0.1*width(I), height(I)/2, map_name + "(x2)", 'Interpreter', 'none', 'HorizontalAlignment', 'center');
    set(h1, 'rotation', 90)
    grid off
    axis off
    rectangle('Position',[0 0 width(I) height(I)], 'Edgecolor', 'r')
    axis equal
    colormap('gray')


    nexttile([1, 3])
    SU_R2E_ANYA = SU{row_name, "R2E_ANYA"};
    SU_R2E_RSP = SU{row_name, "R2E_RSP"};
    SU_R2_ANYA= SU{row_name, "R2_ANYA"};
    SU_R2_RSP = SU{row_name, "R2_RSP"};
    unique_points = SU{row_name, "unique_points"};
    unique_points = unique_points{:};

    semilogy(unique_points, SU_R2E_ANYA{:}, '.-');
    hold on
    semilogy(unique_points, SU_R2E_RSP{:}, 'x-');
    semilogy(unique_points, SU_R2_ANYA{:}, 'o--');
    semilogy(unique_points, SU_R2_RSP{:}, 's--');
    yline(1, ':');
    legend(["R2E vs ANYA", "R2E vs RayScan+", "R2 vs ANYA", "R2 vs RayScan+"], 'Location', 'north' );
    ylim([0.1, inf])
    grid on
    if m == 1
        title("Mean Speed Ups")
    elseif m == height(map_pairs)
        xlabel("Turning Points");
    end
    hold off

    nexttile([1, 3])
    points = T_points{row_name, "points"};
    points = points{:};
    costs = T_costs{row_name, "costs"};
    costs = costs{:};
    plot(points, costs, '.');
    xlim([1, inf]);
    grid on
    hold off
    if m == 1
        title("Path Cost")
    elseif m == height(map_pairs)
        xlabel("Turning Points");
    end

    legend_str = sprintf("$r$ = %.3f\n$\\rho$ = %.1f\\%%", ...
        R.Corr(row_name), ...
        N.Corners(row_name) / N.FreeCells(row_name) * 100);
    text(max(points)*0.95 , max(costs)*0.05, legend_str, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
%     legend(dummy, [legend_str], 'Location', 'southeast', 'Interpreter', 'latex');
end

exportgraphics(TL,'results.pdf','BackgroundColor','none','ContentType','vector');
%% get table of average runtimes
clear all; clc;
script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
load(fullfile(script_dir, "results.mat"));

% print headers
fprintf("Map & Max. Pts. & Max. Cost & $r$ & $\\rho$ (\\%%)")
for var_name = T.Properties.VariableNames
    fprintf(" & %s", var_name{:});
end
fprintf(" \\\\\n\\hline\n")

% print values
for m = 1:height(T)
    fprintf("%s & %d & %.1f & %.3f & %.3f", ...
        replace(T.Properties.RowNames{m}, "_", "\_"), max(T_points.points{m}), max(T_costs.costs{m}), R.Corr(m), N.Corners(m) / N.FreeCells(m) * 100);
    % print average run times
    for a = 1:width(T)
        v = T{m, a};
        v = v{:} / 1000;
        fprintf(" & %.3f", v);
    end
    fprintf(" \\\\\n\\hline\n");
end

%% Get Table of Speed ups
clear all; clc;
script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
load(fullfile(script_dir, "results.mat"));

P = [20, 30];

% print headers
fprintf("Map");
for p = P
    fprintf(" & $g_{%d}$ & E/A & E/P & R/A & R/P", p);
end
fprintf(" \\\\\n\\hline\n")

% print values
for m = 1:height(T)

    fprintf("%s", replace(T.Properties.RowNames{m}, "_", "\_"));
    % print speed ups

    for p = P
        % find average cost for this number of turning points
        idx = T_points.points{m} == p;
        avg_cost = mean(T_costs.costs{m}(idx));
        if isnan(avg_cost)
            fprintf(" & --");
        else
            fprintf(" & %.1f", avg_cost);
        end

        unique_points = SU.unique_points{m};
        idx = unique_points == p;
        su = [SU.R2E_ANYA{m}(idx), SU.R2E_RSP{m}(idx), SU.R2_ANYA{m}(idx), SU.R2_RSP{m}(idx)];
        for s = su
            if isempty(s)
                fprintf(" & --");
            else
                fprintf(" & %.3g", s);
            end
        end
    end
    fprintf(" \\\\\n\\hline\n");
end

%% get results table
A = [];
I = [3, 10, 20, 30];
for m = 1:height(map_pairs)
    t = alg_tables{m};
    a = [mean(t.RR2SE), mean(t.RR2S), mean(t.rsp)];
    unique_tp = unique(t.tp);
    su_rr2se = t.rsp ./ t.RR2SE;
    su_rr2s = t.rsp ./ t.RR2S;
    sutp_rr2se = zeros(numel(unique_tp), 1);
    sutp_rr2s = zeros(numel(unique_tp), 1);
    for i = 1:numel(unique_tp)
        idx = find(t.tp == unique_tp(i));
        sutp_rr2se(i) = mean(su_rr2se(idx));
        sutp_rr2s(i) = mean(su_rr2s(idx));
    end
    for i = I
        tmp = sutp_rr2se(unique_tp == i);
        if (isempty(tmp))
            a = [a, NaN, NaN];
        else
            a = [a, tmp];
            idx = find(t.tp == i);
            a = [a, mean(t.cost(idx))];
        end
    end
    A =[A; a];
end
fprintf("map & rr2se & rr2s & rsp & %i & cost & %i & cost & %i & cost \\\\\n\\hline\n", I(1), I(2), I(3));

[~, I] = sortrows(A, 4);
A = A(I, :);
M = map_pairs(I, :);

for m = 1:height(M)
    fprintf('%s', M(m, 2));
    for a = 1:width(A)
        fprintf(' & %.2f', A(m, a));
    end
    fprintf(" \\\\\n\\hline\n");
end