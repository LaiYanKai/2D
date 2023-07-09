% expt_num must be a number (specify -1 if there is no suffix)
function T = get_scenarios(results_dir, name, algo, expt_num)
T = array2table(zeros(0, 5), 'VariableNames', ["id", "path", "nsec", "cost", "points"]);
if expt_num < 0
    expt_num = "";
else
    expt_num = "." + num2str(expt_num);
end

results_filename = name + "." + algo + expt_num + ".results";
results_path = fullfile(results_dir, results_filename);
if ~isfile(results_path)
    fprintf("File %s does not exist.\n", results_path);
    return
end

fid = fopen(results_path);
nsecs = [];
paths = {};
costs = [];
points = [];

while true
    line = fgetl(fid);
    if line == -1
        break;
    end

    line = str2num(line); % cannot use str2double for some reason
    nsecs = [nsecs; line(1)];
    path = line(2:end);
    path = reshape(path, 2, [])';
    if (isempty(path))
        paths = [paths; nan];
        costs = [costs; nan];
        points = [points; 0];
    else
        paths = [paths; path];
        costs = [costs; sum(vecnorm(diff(path),2, 2))];
        points = [points; height(path)];
    end
end
ids = [0:(numel(nsecs) - 1)]';
T = table(ids, paths, nsecs, costs, points);
T.Properties.VariableNames = ["id", "path", "nsec", "cost", "points"];
disp("Extracted scenarios for " + results_filename);
end