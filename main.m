clear; close all; clc;

% --- PARAMETERS ---
sim.dt = 0.1;
sim.totalTime = 400;
robot.wheelRadius = 0.05;
robot.wheelBase = 0.35;
robot.maxV = 0.6;
robot.maxW = 1.2;
lidar.range = 12.0;           % Increased LIDAR range
lidar.numScans = 360;
lidar.noiseStd = 0.01;
paint.width = 0.6;
paint.spacing = 0.5;
mapResolution = 10;

disp('Draw the room in the top view by clicking on the figure. Press Enter when done.');
[roomBoundary, occMap, wallSegments] = draw_room(mapResolution);

figure('Name', 'User-defined Room & Layout');
show(occMap); hold on;
for i = 1:size(wallSegments, 1)
    plot([wallSegments(i,1), wallSegments(i,3)], [wallSegments(i,2), wallSegments(i,4)], 'k-', 'LineWidth', 2);
end
title('Drawn Environment');

% --- Start robot pose at the CENTROID of the room, facing right (theta=0) ---
roomPoly = polyshape(roomBoundary(:,1), roomBoundary(:,2));
[centerX, centerY] = centroid(roomPoly);
plot(centerX, centerY, 'rx', 'MarkerSize', 15, 'LineWidth', 2); % Mark room center
startPoseTrue = [centerX; centerY; 0];

slamObj = lidarSLAM(lidar.range, mapResolution);
slamObj.LoopClosureThreshold = 210;
slamObj.LoopClosureSearchRadius = 4.0;

truePose = startPoseTrue;
poseEstimate = truePose;
trajTrue = truePose.';
trajEst = poseEstimate.';

planningMap = occMap; % Always use the static map

figure('Name', 'SLAM & Scanning Simulation', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.85]);
hAx = axes; hold on;
show(occMap, 'Parent', hAx); 
title('SLAM Map & Robot'); xlabel('X (m)'); ylabel('Y (m)');

paintMap = false(occMap.GridSize);

% -- PHASE 1: Robot scanning the room by rotating in place --
scanCount = 0; t = 0;
scanDuration = 10; % seconds
disp('Robot is scanning the room...');
while t < scanDuration
    w = robot.maxW/2;
    v = 0;
    dx = v*cos(truePose(3))*sim.dt;
    dy = v*sin(truePose(3))*sim.dt;
    dtheta = w * sim.dt;
    truePose = truePose + [dx; dy; dtheta];
    scan = simulate_lidar_scan(occMap, truePose, lidar);
    noisyRanges = scan.Ranges + lidar.noiseStd * randn(size(scan.Ranges));
    noisyRanges(noisyRanges < 0) = 0;
    scan = lidarScan(noisyRanges, scan.Angles);
    scanCount = scanCount + 1;
    [isUpdated, ~, ~] = slam_update(slamObj, scan);
    trajTrue = [trajTrue; truePose.'];
    trajEst = [trajEst; poseEstimate.'];
    if mod(scanCount, 3) == 0
        if ~exist('hAx','var') || ~isgraphics(hAx)
            hAx = gca;
        end
        cla(hAx); hold(hAx, 'on');
        show(occMap, 'Parent', hAx);
        plot(trajTrue(:,1), trajTrue(:,2), '-b', 'LineWidth', 1, 'Parent', hAx);
        quiver(truePose(1), truePose(2), cos(truePose(3))*0.3, sin(truePose(3))*0.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'Parent', hAx);
        title(hAx, sprintf('Scanning... t=%.1f', t));
        drawnow;
    end
    pause(0.01);
    t = t + sim.dt;
end

% --- MOVE FROM CENTER TO CLOSEST BOUNDARY POINT ---
diffs = roomBoundary - repmat([centerX centerY], size(roomBoundary,1), 1);
dists = sqrt(sum(diffs.^2,2));
[~, closestIdx] = min(dists);
targetWallPt = roomBoundary(closestIdx, :);

disp('Robot moving from center to boundary...');
reachedWall = false;
controllerState = [];
while ~reachedWall
    % Go to wall point using diffdrive controller
    localGoal = targetWallPt';
    [v, w, controllerState] = diffdrive_controller(truePose, localGoal, robot, controllerState);
    v = max(min(v, robot.maxV), -robot.maxV);
    w = max(min(w, robot.maxW), -robot.maxW);
    dx = v*cos(truePose(3))*sim.dt;
    dy = v*sin(truePose(3))*sim.dt;
    dtheta = w * sim.dt;
    truePose = truePose + [dx; dy; dtheta];
    trajTrue = [trajTrue; truePose.'];
    if ~exist('hAx','var') || ~isgraphics(hAx)
        hAx = gca;
    end
    cla(hAx); hold(hAx, 'on');
    show(occMap, 'Parent', hAx);
    plot(trajTrue(:,1), trajTrue(:,2), '-b', 'LineWidth', 1, 'Parent', hAx);
    quiver(truePose(1), truePose(2), cos(truePose(3))*0.3, sin(truePose(3))*0.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'Parent', hAx);
    plot(targetWallPt(1), targetWallPt(2), 'ro', 'MarkerSize', 12, 'LineWidth',2, 'Parent', hAx);
    title(hAx, 'Moving to Room Boundary...');
    drawnow;

    % Check if robot has reached the wall
    if norm(truePose(1:2) - targetWallPt') < 0.1
        reachedWall = true;
    end
end

% -- PHASE 2: Robot patrols the interior boundary --
disp('Robot will patrol the wall interior...');
wallPath = plan_wall_following(roomBoundary, paint.spacing);
trajTrue = truePose.';

for i=1:size(wallPath,1)
    localGoal = wallPath(i,:)';
    [v, w, controllerState] = diffdrive_controller(truePose, localGoal, robot, []);
    v = max(min(v, robot.maxV), -robot.maxV);
    w = max(min(w, robot.maxW), -robot.maxW);
    dx = v*cos(truePose(3))*sim.dt;
    dy = v*sin(truePose(3))*sim.dt;
    dtheta = w * sim.dt;
    truePose = truePose + [dx; dy; dtheta];

    trajTrue = [trajTrue; truePose.'];
    paintMap = paint_simulator(paintMap, occMap, truePose, paint);

    if mod(i, 3) == 0
        if ~exist('hAx','var') || ~isgraphics(hAx)
            hAx = gca;
        end
        cla(hAx); hold(hAx, 'on');
        paintedIdx = find(paintMap);
        show(occMap, 'Parent', hAx);
        if ~isempty(paintedIdx)
            coords = grid2world(occMap, paintedIdx);
            scatter(coords(:,1), coords(:,2), 10, 'filled', 'MarkerFaceAlpha', 0.6, 'Parent', hAx);
        end
        plot(trajTrue(:,1), trajTrue(:,2), '-b', 'LineWidth', 1, 'Parent', hAx);
        quiver(truePose(1), truePose(2), cos(truePose(3))*0.3, sin(truePose(3))*0.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'Parent', hAx);
        title(hAx, sprintf('Wall Patrol Step %d / %d', i, size(wallPath,1)));
        drawnow;
    end
end

fprintf('Simulation finished. Painted cells: %d\n', sum(paintMap(:)));


% --- HELPER FUNCTIONS ---

function [boundary, occMap, wallSegments] = draw_room(mapResolution)
    figure('Name','Draw the Room Layout');
    axis([0 5 0 5]); grid on; hold on;
    title('Click to define the room boundary (clockwise or CCW). Press Enter when done.');
    [x, y] = getline; % User draws room by clicking
    boundary = [x, y];

    % Remove all duplicate points (not just consecutive)
    [~, uniqueIdx] = unique(boundary, 'rows', 'stable');
    boundary = boundary(uniqueIdx, :);

    % Remove consecutive points that are nearly identical 
    diffPts = diff(boundary);
    distPts = sqrt(sum(diffPts.^2,2));
    boundary = boundary([true; distPts > 1e-3], :);

    % Force closure
    if norm(boundary(1,:) - boundary(end,:)) > 1e-3
        boundary = [boundary; boundary(1,:)];
    end

    wallSegments = [boundary(1:end-1,:) boundary(2:end,:)];    
    for k = 1:size(wallSegments,1)
        plot(wallSegments(k,[1,3]), wallSegments(k,[2,4]), 'k-', 'LineWidth', 2);
    end

    occMap = occupancyMap(5, 5, mapResolution);
    polyin = polyshape(boundary(:,1), boundary(:,2));
    [X, Y] = meshgrid(linspace(0,5,5*mapResolution), linspace(0,5,5*mapResolution));
    in = isinterior(polyin,X(:),Y(:));
    occMap.setOccupancy([X(:),Y(:)], ~in); 
end

function wallPath = plan_wall_following(boundary, spacing)
    offset = 0.1; % Offset inward for wall following path
    wallPath = [];
    for i = 1:(size(boundary,1)-1)
        p1 = boundary(i,:);
        p2 = boundary(i+1,:);
        dp = p2 - p1;
        L = norm(dp);
        nPts = ceil(L / spacing);
        for k = 0:nPts
            pt = p1 + k/nPts*dp;
            % Compute inward normal
            normal = dp / L;
            normal = [-normal(2), normal(1)]; % 90 deg CCW rotation
            pt_in = pt + offset * normal;
            wallPath = [wallPath; pt_in];
        end
    end
end

% --- You should have these helper functions implemented in your path: ---
% - diffdrive_controller
% - simulate_lidar_scan
% - paint_simulator
% - slam_update
