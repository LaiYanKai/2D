% Scale Map twice, and convert pfarc into scen
clear all
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
sc = 2;

for p = 1:size(map_pairs, 1)
    % convert map to two times
    f_dir = map_pairs(p, 1);
    f_name = map_pairs(p, 2);
    f_pfarc = fullfile("..","rayscan","scenarios", f_dir, f_name + ".pfarc");
    fprintf(" pfarc\t%s\n", f_pfarc);
    f_scen = fullfile("data", f_dir, f_name + "_scale2.map.scen");
    fprintf("  scen\t%s\n", f_scen);
    f_mapname =  f_name + "_scale2.map";
    f_mapold = fullfile("data", f_dir, f_name + ".map");
    fprintf("mapold\t%s\n", f_mapold);
    f_map = fullfile("data", f_dir, f_mapname);
    fprintf("mapnew\t%s\n", f_map);

    % extract mp data
    if (~isfile(f_mapold))
        fprintf("Map File %s does not exist. \n", f_map);
        continue;
    else
        fid_mapold = fopen(f_mapold,'r');
        fid_map = fopen(f_map, "w");
        map_size = fscanf(fid_mapold, "type octile\nheight %d\nwidth %d\nmap\n");
        fprintf(fid_map, "type octile\nheight %d\nwidth %d\nmap\n", map_size(1)*sc, map_size(2)*sc);

        for i = 1:map_size(1)
            mp_line = fscanf(fid_mapold, "%1s", [map_size(2), 1]);
            mp_sc = repelem(mp_line, sc);
            for s = 1:sc
                fprintf(fid_map, "%s\n", mp_sc);
            end
        end
        fclose(fid_mapold);
    end

    % convert pfarc to scens
    map_size = map_size * sc;
    if (~isfile(f_pfarc))
        fprintf("File %s does not exist. \n", f_pfarc);
        continue;
    else
        lines_pfarc = readlines(f_pfarc);
        fid_scen = fopen(f_scen, "w");
        fprintf(fid_scen, "version 1\n");
        for i=1:numel(lines_pfarc)
            l = split(lines_pfarc(i), " ");
            if (l(1) == "instance")
                si = l(3);
                sj = map_size(1) - str2double(l(4));
                ti = l(6);
                tj = map_size(1) - str2double(l(7));
                cost = l(9);
                fprintf(fid_scen, "0\t%s\t%d\t%d\t%s\t%d\t%s\t%d\t%s\n", f_mapname, map_size(1), map_size(2), si, sj, ti, tj, cost);
            end
        end

    end
    disp("---");
end