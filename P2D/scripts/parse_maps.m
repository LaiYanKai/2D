
function [M, C] = parse_maps(directory, name, display)
M = [];
C = [];
% extract mp data
fname = fullfile(directory, name + ".map");
if ~isfile(fname)
    fprintf("File %s does not exist.\n", fname);
    return
end
fprintf("Opening %s\n", fname);
fileID = fopen(fname,'r');
ni = fscanf(fileID, "type octile\nheight %d\n");
nj = fscanf(fileID, "width %d\nmap\n");
% get mp
mp_str = fscanf(fileID,"%1s",[nj, ni]);
fclose(fileID);

% convert mp to numeric format
mp_str = mp_str';
mp = zeros(ni, nj);
for i = 1:ni
    for j = 1:nj
        s = mp_str(i, j);
        if s ~= '.'
            mp(i, j) = 1;
        end
    end
end
mp = reshape(mp', 1, []);

% append to struct
M = struct('num_i', ni, 'num_j', nj, 'name', '', 'path', fname, 'mp', mp);
[~, M.name, ~] = fileparts(fname);

C = ~logical(reshape(M.mp, M.num_j, M.num_i));
C = double(C);

if ~display
    return
end
imagesc(C, "XData", 0.5, "YData", 0.5);
set(gca,'YDir','normal')
hold on
rectangle('Position',[0 0 M.num_i M.num_j], 'Edgecolor', 'r')
axis on
grid on
h = gca;
h.XAxis.MinorTickValues = 0:(M.num_i);
h.XAxis.MinorTick = 'On';
h.YAxis.MinorTickValues = 0:(M.num_j);
h.YAxis.MinorTick = 'On';
h.XLabel.String = 'X';
h.YLabel.String = 'Y';
grid(gca, 'minor')
colormap('gray')
axis equal

% for data tips
hold on
% cpath = fullfile("P2D", "scripts", "corners", strcat(map_pair(2), ".mat"));
[X,Y] = findCorners(M);
% save(cpath, 'X', 'Y');
% load(cpath, 'X', 'Y');
sh = scatter(X, Y);
sh.MarkerFaceColor = [0,0,0];
sh.MarkerFaceAlpha = 0.;
sh.MarkerEdgeAlpha = 0.;
hold off

% modify the datatips to be more succinct
dcm = datacursormode(gcf);
dcm.Enable = 'off';
% dcm.SnapToDataVertex = 'off';
dcm.UpdateFcn = @displayCoordinates;
end

function txt = displayCoordinates(~,info)
x = info.Position(1);
y = info.Position(2);
txt = ['' num2str(x) ', ' num2str(y)];
end

function [X,Y] = findCorners(M)
X = [];
Y = [];

% find corners for all four corners
if ~M.mp(1)
    X = 0;
    Y = 0;
end
if ~M.mp(M.num_j)
    X = [X, 0];
    Y = [Y, M.num_j];
end
if ~M.mp((M.num_i-1)*M.num_j+1)
    X = [X, M.num_i];
    Y = [Y, 0];
end
if ~M.mp(M.num_i*M.num_j)
    X = [X, M.num_i];
    Y = [Y, M.num_j];
end

% find corners for map bounds
for j = 1:(M.num_j-1)
    if M.mp(j) ~= M.mp(j+1)
        X = [X, 0];
        Y = [Y, j];
    end
end
k = (M.num_i-1)*M.num_j;
for j = 1:(M.num_j-1)
    if M.mp(j+k) ~= M.mp(j+k+1)
        X = [X, M.num_i];
        Y = [Y, j];
    end
end
for i = 0:(M.num_i-2)
    if M.mp(i*M.num_j+1) ~= M.mp((i+1)*M.num_j+1)
        X = [X, i+1];
        Y = [Y, 0];
    end
end
for i = 1:(M.num_i-1)
    if M.mp(i*M.num_j) ~= M.mp((i+1)*M.num_j)
        X = [X, i];
        Y = [Y, M.num_j];
    end
end

% find corners for everywhere else
b = 1;
for i = 0:(M.num_i-2)
    for j = 0:(M.num_j-2)
        SE = M.mp(b);
        NE = M.mp(b + M.num_j);
        NW = M.mp(b + M.num_j + 1);
        SW = M.mp(b + 1);

        if ((SE == NW || SW == NE) && (NE ~= SE || NW ~= SW)) % corner
            X = [X, i+1];
            Y = [Y, j+1];
        end
        b = b + 1;
    end
    b = b + 1;
end
end