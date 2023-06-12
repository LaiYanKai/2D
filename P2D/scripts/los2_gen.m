
clear all; clc
THRES = 1e-8;

src = [12.22, 1.33];
tgt = [-12.2, -11.2];

diffX = tgt - src;
absX = abs(diffX);

cflag = absX(1) > absX(2);

diffZ = conv(cflag, diffX);
absZ = abs(diffZ);
dZ = int32(sign(diffZ));

srcZ = conv(cflag, src);
tgtZ = conv(cflag, tgt);
tgtFloorZ = int32(floor(tgtZ)); % store as int
floorZ = int32(floor(srcZ)); % store as int
prevS = srcZ(2);

psiZ = [int32(dZ(1) > 0) * dZ(1), int32(dZ(2) > 0) * dZ(2)];
cmp = (double(floorZ(1)) + double(psiZ(1)) - srcZ(1)) * absZ(2)/diffZ(1) + double(dZ(2)) * double(psiZ(2));
changeS = diffZ(2) / absZ(1);

path = src;
step = 0;
while (~isequal(floorZ, tgtFloorZ))
    step = step + 1;

    S = changeS * step + srcZ(2);
    floorS = floor(S);
    if (int32(floorS) ~= floorZ(2))
        % short incremented
        cmpS = double(dZ(2)) * (double(floorZ(2)) - prevS);

        if (cmpS - THRES > cmp)
            % pass thru large first
            floorZ(1) = floorZ(1) + dZ(1);
            path = [path; conv(cflag, floorZ)];
            fprintf("small1: [%f,%f]\n", floorZ);
            if (isequal(floorZ, tgtFloorZ))
                break; % reached destination
            end
            floorZ(2) = floorZ(2) + dZ(2);
            fprintf("small2: [%f,%f]\n", floorZ);
        elseif (cmpS + THRES < cmp)
            % pass thru small first
            floorZ(2) = floorZ(2) + dZ(2);
            path = [path; conv(cflag, floorZ)];
            fprintf("long1: [%f,%f]\n", floorZ);
            if (isequal(floorZ, tgtFloorZ))
                break;
            end
            floorZ(1) = floorZ(1) + dZ(1);
            fprintf("long2: [%f,%f]\n", floorZ);
        else
            % pass thru both at same time
            floorZ = floorZ + dZ;
            fprintf("eq: [%f,%f]\n", floorZ);
        end
    else
        % no change in short
        floorZ(1) = floorZ(1) + dZ(1);
        fprintf("nc: [%f,%f]\n", floorZ);
    end
    path = [path; conv(cflag, floorZ)];  
    prevS = S;
end

disp(path);


figure(1)
clf;
hold on
for (i = 1:size(path,1))
    rectangle('Position',[path(i,1),path(i,2),1,1],'FaceColor',[.8 .8 .8])
end
plot ([src(1), tgt(1)], [src(2), tgt(2)], 'x-r');
hold off


function A = conv(absDx_Gt_absDy, B)
    if absDx_Gt_absDy 
        A = [B(1), B(2)];
    else
        A = [B(2), B(1)];
    end
end