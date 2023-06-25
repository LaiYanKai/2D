%% Import and collate results, and save them
clear all; clc; close all

algs = ["R2E", "R2"]; % make sure the first alg returns the optimal path, and the optimal path does not contain more than 3 consecutive colinear points anywhere
expt_nums = [0:9];
map_pairs = [
    "dao", "arena";
    "bg512", "AR0709SR";
    "bg512", "AR0504SR";
    "bg512", "AR0014SR";
    "bg512", "AR0304SR";
    "bg512", "AR0702SR";
    "bg512", "AR0205SR";
    "bg512", "AR0602SR";
    "bg512", "AR0603SR";
    "street", "Denver_2_1024";
    "street", "NewYork_0_1024";
    "street", "Shanghai_2_1024";
    "street", "Shanghai_0_1024";
    "street", "Sydney_1_1024";
    "da2", "ht_mansion2b";
    "da2", "ht_0_hightown";
    "dao", "hrt201n";
    "room", "32room_000";
    "room", "16room_000";
    ];


script_path = matlab.desktop.editor.getActiveFilename;
[script_dir, ~, ~] = fileparts(script_path);
addpath(script_dir);

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
map_pairs = map_pairs(:, 1) + "/" + map_pairs(:, 2);
T_nsecs = cell2table(T_nsecs, "RowNames", map_pairs, "VariableNames", algs);
T_costs = cell2table(T_costs, "RowNames", map_pairs, "VariableNames", ["costs"]);
T_points = cell2table(T_points, "RowNames", map_pairs, "VariableNames", ["points"]);

save(fullfile(script_dir, "results.mat"), "T_nsecs", "T_costs", "T_points");
%% 
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
    % gather points with same number of speed ups
    points = T_points{m, 1};
    points = points{:};
    max_points = max(points);
    SU_R2E_ANYA = zeros(max_points, 1);
    SU_R2_ANYA = zeros(max_points, 1);
    SU_R2E_RSP = zeros(max_points, 1);
    SU_R2_RSP = zeros(max_points, 1);
    for p = 2:max_points
        idx = points == p;
        nsecs_R2 = T_nsecs.R2{m};
        nsecs_R2E = T_nsecs.R2E{m};
        SU_R2E_ANYA(p) = mean(nsecs_R2(idx) ./ nsecs_R2E(idx));
    end
    SU{m, 1} = SU_R2E_ANYA;
    SU{m, 2} = SU_R2_ANYA;
    SU{m, 3} = SU_R2E_RSP;
    SU{m, 4} = SU_R2_RSP;
        
    % collect average runtimes for each algo
    for a = 1:width(T)
        data = T{m ,a};
        T{m, a} = {mean(data{:})};
    end
end





% %% read .txt files from rsp
% clear all; close all; clc
% addpath(fileparts(which(mfilename)))
%
% rsp_tables = cell(height(map_pairs), 1);
% num_runs = 10;
% for m = 1:height(map_pairs)
%     map_name = map_pairs(m, 2);
%     map_dir = map_pairs(m, 1);
%
%     T = table();
%     for i = 1:num_runs
%         t = readtable(fullfile("results", map_dir, ...
%             map_name + "_" + num2str(i) + ".txt"));
%         T.("run" + num2str(i)) = t.searchNs / 1000; % conv times to us
%     end
%     tmp = table2array(T);
%     T.meanus = mean(tmp, 2);
%     T.id = t.vID + 1; % +1 for rsp
%     T.map = repmat(map_name, [height(T), 1]);
%     T.alg = repmat("rsp", [height(T), 1]);
%     T = sortrows(T, "id");
%     rsp_tables{m} = T;
%
%     disp("Tabulated Rsp " + map_name);
%
%
% end
% %% read logs from .show_results
% % convert to tab delimited
% % header: id	alg	map	run	si	sj	ti	tj	cost	tp	us
% t = readtable("results_rr2_.log");
% algs = ["RR2SE", "RR2S", "RR2E", "RR2"];
%
% rr2_tables = cell(height(map_pairs), numel(algs));
% for a = 1:numel(algs)
%     tmp = t(t.alg == algs(a), :); % filter by algo;
%     for m = 1:height(map_pairs)
%         map_name = map_pairs(m, 2);
%         tmp2 = tmp(tmp.map == map_name, :); % filter by map
%         T = table();
%         for i = 1:num_runs
%             tmp3 = tmp2(tmp2.run == i, :); % filter by run
%             T.("run" + num2str(i)) = tmp3.us;
%         end
%         tmp2 = table2array(T);
%         T.meanus = mean(tmp2, 2);
%         T.id = tmp3.id;
%         T.map = repmat(map_name, [height(T), 1]);
%         T.alg = repmat(algs(a), [height(T), 1]);
%         T.cost = tmp3.cost; % copy cost once.
%         T.tp = tmp3.tp; % copy tp once.
%         T = sortrows(T, "id");
%         rr2_tables{m, a} = T;
%         disp("Tabulated " + algs(a) + " " + map_name);
%     end
% end
% %% combine all tables with average speed
% alg_tables = cell(height(map_pairs), 1);
%
% for m = 1:height(map_pairs)
%     T = table();
%     map_name = map_pairs(m, 2);
%
%     % find map in rsp and fill T
%     for i = 1:height(rsp_tables)
%         tmp = rsp_tables{i};
%         if strcmp(tmp{1, "map"}, map_name)
%             T.rsp = tmp.meanus;
%             T.id = tmp.id;
%             T.map = tmp.map;
%             break
%         end
%     end
%
%     for a = 1:width(rr2_tables)
%         for i = 1:height(rr2_tables)
%             tmp = rr2_tables{i, a};
%             if strcmp(tmp{1, "map"}, map_name)
%                 if (~isequal(tmp.id, T.id) || ~isequal(tmp.map, T.map))
%                     disp("tables not equal");
%                 end
%                 T.(tmp{1, "alg"}) = tmp.meanus;
%                 if (a == 1) % copy info once
%                     T.tp = tmp.tp;
%                     T.cost = tmp.cost;
%                 end
%                 break
%             end
%         end
%     end
%
%     alg_tables{m} = T;
% end
% %% plot
% m = 18;  %2(AR0205SR) %5(AR0602SR)  %6(AR0603SR) %10 (Denver 2) %11(NewYork)
% % 19 16room % 18 32room % 16 ht_0_hightown  %12(Shanghai_2_1024)
% %18 (32room), 15(ht_mansion2b) 11(NewYork) 4(AR0014SR) 5(AR0304SR)
% maps = [18, 15, 4, 11, 12];
% figure (1)
% set(gcf, 'Position',  [100, 100, 1115, 140*numel(maps)]);
% T = tiledlayout(numel(maps), 7,'TileSpacing','Compact','Padding','None');
%
% mm = 0;
% for m = maps
%     mm = mm + 1;
%     map_name = map_pairs(m, 2);
%     map_dir = map_pairs(m, 1);
%     disp(map_name);
%     t = alg_tables{m};
%
%
%     fprintf('avg us rr2se: %f\n', mean(t.RR2SE));
%     fprintf('avg us rr2s: %f\n', mean(t.RR2S));
%     fprintf('avg us rr2e: %f\n', mean(t.RR2E));
%     fprintf('avg us rr2: %f\n', mean(t.RR2));
%     fprintf('avg us rsp: %f\n', mean(t.rsp));
%
%     costs = t.cost;
%     tp = t.tp;
%     su_rr2se = t.rsp ./ t.RR2SE;
%     su_rr2s = t.rsp ./ t.RR2S;
%     su_rr2e = t.rsp ./ t.RR2E;
%     su_rr2 = t.rsp ./ t.RR2;
%
%     [~, idx] = sort(costs);
% %     n = 100;
% %     buckets = ceil(numel(costs)/n);
% %     avg_costs = zeros(buckets, 1);
% %     avg_su_rr2se = zeros(buckets, 1);
% %     avg_su_rr2s = zeros(buckets, 1);
% %     avg_su_rr2e = zeros(buckets, 1);
% %     avg_su_rr2 = zeros(buckets, 1);
% %     for b = 1:buckets
% %         idx2 = idx(((b-1)*n+1):min(b*n, numel(costs)));
% %         avg_costs(b) = mean(costs(idx2));
% %         avg_su_rr2se(b) = mean(su_rr2se(idx2));
% %         avg_su_rr2s(b) = mean(su_rr2s(idx2));
% %         avg_su_rr2e(b) = mean(su_rr2e(idx2));
% %         avg_su_rr2(b) = mean(su_rr2(idx2));
% %     end
%
%     % figure (1)
%     % % semilogy(costs, su_rr2es, '.');
%     % % semilogy(costs, su_rr2s, '.');
%     % % semilogy(costs, su_rr2e, '.');
%     % % semilogy(costs, su_rr2, '.');
%     % semilogy(avg_costs, avg_su_rr2se, 's-');
%     % hold on
%     % semilogy(avg_costs, avg_su_rr2s, 's--');
%     % % semilogy(avg_costs, avg_su_rr2e, 'x-');
%     % % semilogy(avg_costs, avg_su_rr2, 'x--');
%     % % legend(["rr2se", "rr2s", "rr2e", "rr2", 'rr2es avg', 'rr2s avg', 'rr2e avg', 'rr2 avg']);
%     % legend(["RR2E", "RR2"]);
%     % yline(1, 'k-.');
%     % grid on
%     % hold off
%
%     % calc average speedup per tp
%     unique_tp = unique(tp);
%     sutp_rr2se = zeros(numel(unique_tp), 1);
%     sutp_rr2s = zeros(numel(unique_tp), 1);
%     sutp_rr2e = zeros(numel(unique_tp), 1);
%     sutp_rr2 = zeros(numel(unique_tp), 1);
%     for i = 1:numel(unique_tp)
%         idx = find(tp == unique_tp(i));
%         sutp_rr2se(i) = mean(su_rr2se(idx));
%         sutp_rr2s(i) = mean(su_rr2s(idx));
%         sutp_rr2e(i) = mean(su_rr2e(idx));
%         sutp_rr2(i) = mean(su_rr2(idx));
%     end
%
%     % semilogy(tp, su_rr2es, 'o');
%     % hold on
%     % semilogy(tp, su_rr2s, 'x');
%     % % semilogy(tp, su_rr2e, '.');
%     % % semilogy(tp, su_rr2, '.');
%     %
%     % semilogy(unique_tp, sutp_rr2es, '.-');
%     % semilogy(unique_tp, sutp_rr2s, '.--');
%     % % semilogy(unique_tp, sutp_rr2e, 'x-');
%     % % semilogy(unique_tp, sutp_rr2, 'x--');
%     %
%     % yline(1, 'k-.')
%     % % legend(["rr2es", "rr2s", "rr2e", "rr2", 'rr2es avg', 'rr2s avg', 'rr2e avg', 'rr2 avg']);
%     % legend(["rr2es", "rr2s", "rr2es avg", "rr2s avg"]);
%     % grid on
%     % hold off
%
%
%     nexttile
%     M = parse_maps(fullfile("data", map_dir), map_name);
%     C = ~logical(reshape(M.mp, M.num_j, M.num_i));
%     C = double(C);
%     ih = imagesc(C, "XData", 0.5, "YData", 0.5);
% %     ylabel(map_name, 'Interpreter','none')
%     h1 = text(-0.1*M.num_i, M.num_j/2, map_name, 'Interpreter', 'none', 'HorizontalAlignment', 'center');
%     set(h1, 'rotation', 90)
%     grid off
%     axis off
%     rectangle('Position',[0 0 M.num_i M.num_j], 'Edgecolor', 'r')
%     axis equal
%     colormap('gray')
%
%
%     nexttile([1, 3])
%     plot(unique_tp, sutp_rr2se, '.-');
%     hold on
%     plot(unique_tp, sutp_rr2s, '.--');
%     yline(1, '--');
%     legend(["RR2E", "RR2"], 'Location', 'north' );
%     ylim([0, inf])
%     grid on
%     if mm == 1
%         title("Mean Speed Ups")
%     elseif mm == numel(maps)
%         xlabel("Turning Points");
%     end
%
%     hold off
%
%     nexttile([1, 3])
%     plot(tp, costs, '.');
%     grid on
%     hold off
%     if mm == 1
%         title("Path Cost (Num. Cells)")
%     elseif mm == numel(maps)
%         xlabel("Turning Points");
%     end
% end
%
% exportgraphics(T,'results.pdf','BackgroundColor','none','ContentType','vector');
%
% A = [];
% I = [3, 10, 20, 30];
% for m = 1:height(map_pairs)
%     t = alg_tables{m};
%     a = [mean(t.RR2SE), mean(t.RR2S), mean(t.rsp)];
%     unique_tp = unique(t.tp);
%     su_rr2se = t.rsp ./ t.RR2SE;
%     su_rr2s = t.rsp ./ t.RR2S;
%     sutp_rr2se = zeros(numel(unique_tp), 1);
%     sutp_rr2s = zeros(numel(unique_tp), 1);
%     for i = 1:numel(unique_tp)
%         idx = find(t.tp == unique_tp(i));
%         sutp_rr2se(i) = mean(su_rr2se(idx));
%         sutp_rr2s(i) = mean(su_rr2s(idx));
%     end
%     for i = I
%         tmp = sutp_rr2se(unique_tp == i);
%         if (isempty(tmp))
%             a = [a, NaN, NaN];
%         else
%             a = [a, tmp];
%             idx = find(t.tp == i);
%             a = [a, mean(t.cost(idx))];
%         end
%     end
%     A =[A; a];
% end
% fprintf("map & rr2se & rr2s & rsp & %i & cost & %i & cost & %i & cost \\\\\n\\hline\n", I(1), I(2), I(3));
%
% [~, I] = sortrows(A, 4);
% A = A(I, :);
% M = map_pairs(I, :);
%
% for m = 1:height(M)
%     fprintf('%s', M(m, 2));
%     for a = 1:width(A)
%         fprintf(' & %.2f', A(m, a));
%     end
%     fprintf(" \\\\\n\\hline\n");
% end
%
% function M = parse_maps(directory, name, flat)
%
% if nargin == 1
%     fnames = dir(fullfile(directory, '/*.map'));
%     fnames = {fnames.name};
%     fnames = convertCharsToStrings(fnames);
%     flat = true;
% elseif nargin == 2
%     fnames = convertCharsToStrings(name) + ".map";
%     flat = true;
% else
%     fnames = convertCharsToStrings(name) + ".map";
% end
% M = [];
% for f = 1:length(fnames)
%     % extract mp data
%     fname = fullfile(directory, fnames(f));
%     if ~isfile(fname)
%         fprintf("File %s does not exist.\n", fname);
%         continue
%     end
%     fprintf("Opening %s\n", fname);
%     fileID = fopen(fname,'r');
%     ni = fscanf(fileID, "type octile\nheight %d\n");
%     nj = fscanf(fileID, "width %d\nmap\n");
%     % get mp
%     mp_str = fscanf(fileID,"%1s",[nj, ni]);
%     fclose(fileID);
%
%     % convert mp to numeric format
%     mp_str = mp_str';
%
%
%     mp = zeros(ni, nj);
%     for i = 1:ni
%         for j = 1:nj
%             s = mp_str(i, j);
%             if s ~= '.'
%                 mp(i, j) = 1;
%             end
%         end
%     end
%
%     if flat == true
%         mp = reshape(mp', 1, []);
%     end
%
%     % append to struct
%     m = struct('num_i', ni, 'num_j', nj, ...
%         'name', '', 'path', fname, 'mp', mp);
%     [~, m.name, ~] = fileparts(fname);
%     M = [M, m];
% end
% end