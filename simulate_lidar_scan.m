function scan = simulate_lidar_scan(occMap, pose, lidar)
% simulate_lidar_scan Simulate a 2D lidar scan from occupancyMap at given pose.
% pose = [x;y;theta]
% lidar fields: range, numScans

angles = linspace(-pi, pi, lidar.numScans)';
ranges = lidar.range * ones(size(angles));
occMatrix = occupancyMatrix(occMap);  % get the occupancy grid matrix

for i=1:length(angles)
    a = angles(i) + pose(3);
    t = linspace(0, lidar.range, 200);
    xs = pose(1) + t*cos(a);
    ys = pose(2) + t*sin(a);
    indices = world2grid(occMap, [xs' ys']);
    ix = round(indices(:,1));
    iy = round(indices(:,2));
    valid = ix>=1 & iy>=1 & ix<=occMap.GridSize(2) & iy<=occMap.GridSize(1);
    hitIdx = [];
    for k=1:length(t)
        if ~valid(k), break; end
        if occMatrix(iy(k), ix(k)) > 0.5
            hitIdx = k;
            break;
        end
    end
    if ~isempty(hitIdx)
        ranges(i) = t(hitIdx);
    else
        ranges(i) = lidar.range;
    end
end

scan = lidarScan(ranges, angles);

end
