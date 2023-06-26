%% Import and collate results, and save them
clear all; clc; close all

algs = ["R2E", "R2", "ANYA"]; % make sure the first alg returns the optimal path, and the optimal path does not contain more than 3 consecutive colinear points anywhere
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
save(fullfile(script_dir, "results.mat"), "T_nsecs", "T_costs", "T_points");

%% import RSP files
RSP_nsecs = cell(height(map_pairs), 1);
for m = 1:height(map_pairs)
    name = extractBefore(map_pairs(m, 2), "_scale2");
    avg_nsecs = [];
    for i = (expt_nums + 1) % started from 1
        t = readtable(fullfile("..", "rayscan", "results", map_pairs(m,1), name + "_" + num2str(i) + ".txt"));
        avg_nsecs = [avg_nsecs, t.searchNs];
    end
    RSP_nsecs{m} = mean(avg_nsecs, 2);
    disp("Tabulated Rsp " + name);
end

T_nsecs.("RSP") = RSP_nsecs;
save(fullfile(script_dir, "results.mat"), "T_nsecs", "T_costs", "T_points");
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
        nsecs_ANYA = T_nsecs.ANYA2B{m};
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
end
SU = cell2table(SU, "RowNames", T_nsecs.Properties.RowNames, ...
    "VariableNames", ["unique_points", "R2E_ANYA", "R2_ANYA", "R2E_RSP", "R2_RSP"]);
save(fullfile(script_dir, "results.mat"), "T", "T_nsecs", "T_costs", "T_points", "SU");
%% plot
clear all; close all; clc
script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);
load(fullfile(script_dir, "results.mat"));

map_pairs = [
    "random", "random512-10-1"
%     "room", "32room_000";
%     "da2", "ht_mansion2b";
%     "bg512", "AR0014SR";
%     "street", "NewYork_0_1024";
%     "street", "Shanghai_2_1024"
    ];
figure (1)
set(gcf, 'Position',  [100, 100, 1115, 140*numel(map_pairs)]);
TL = tiledlayout(height(map_pairs), 7,'TileSpacing','Compact','Padding','None');

for m = 1:height(map_pairs)
    map_name = map_pairs(m, 2);
    nexttile
    [M, C] = parse_maps(fullfile("data", map_pairs(m, 1)), map_name, false);
    ih = imagesc(C, "XData", 0.5, "YData", 0.5);
    ylabel(map_name, 'Interpreter','none')
    h1 = text(-0.1*M.num_i, M.num_j/2, map_name, 'Interpreter', 'none', 'HorizontalAlignment', 'center');
    set(h1, 'rotation', 90)
    grid off
    axis off
    rectangle('Position',[0 0 M.num_i M.num_j], 'Edgecolor', 'r')
    axis equal
    colormap('gray')


    nexttile([1, 3])
    row_name = map_pairs(m, 1) + "/" + map_pairs(m, 2);
    SU_R2E_RSP = SU{row_name, "R2E_RSP"};
    SU_R2_RSP = SU{row_name, "R2_RSP"};
    SU_R2E_ANYA = SU{row_name, "R2E_ANYA"};
    SU_R2_ANYA= SU{row_name, "R2_ANYA"};
    unique_points = SU{row_name, "unique_points"};
    unique_points = unique_points{:};

    plot(unique_points, SU_R2E_RSP{:}, 'x-');
    hold on
    plot(unique_points, SU_R2_RSP{:}, 's--');
    plot(unique_points, SU_R2E_ANYA{:}, '.-');
    plot(unique_points, SU_R2_ANYA{:}, 'o--');
    yline(1, ':');
    legend(["R2E vs RayScan+", "R2 vs RayScan+", "R2E vs ANYA", "R2 vs ANYA"], 'Location', 'north' );
    ylim([0, inf])
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
    grid on
    hold off
    if m == 1
        title("Path Cost (Num. Cells)")
    elseif m == height(map_pairs)
        xlabel("Turning Points");
    end
    r = corrcoef(points, costs);
    legend(["Correlation Coefficient = " + num2str(r(1, 2))], 'Location', 'southeast');
end

exportgraphics(TL,'results.pdf','BackgroundColor','none','ContentType','vector');
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