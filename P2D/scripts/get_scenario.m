function S = get_scenario(results_dir, name, algo, scen)
S = struct('nsec', nan, 'cost', nan, 'path', []);
if scen < 0
    disp("Scen must be >= 0")
    return
end
results_filename = name + "." + algo + ".results";
results_path = fullfile(results_dir, results_filename);
if ~isfile(results_path)
    fprintf("File %s does not exist.\n", results_path);
    return
end
fid = fopen(results_path);
for i = 1:scen
    fgetl(fid);
end
line = fgetl(fid);
if (line == -1)
    disp("Number of scenarios exceeded. Nothing to return")
    return;
end

line = str2num(line); % cannot use str2double for some reason
S.nsec = line(1);
S.path = line(2:end);
S.path = reshape(S.path, 2, [])';
S.cost = sum(vecnorm(diff(S.path),2, 2));
fprintf("%s  Scen(%d)\n\tnsec(%d) cost(%f) points(%d)\n\tpath( ", results_filename, scen, S.nsec, S.cost, height(S.path));
for i = 1:height(S.path)
    coord = S.path(i, :);
    fprintf("%d,%d; ", coord(1), coord(2));
end
fprintf(")\n");
end