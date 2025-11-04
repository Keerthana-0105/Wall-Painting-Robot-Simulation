function paintMap = paint_simulator(paintMap, occMap, pose, paint)
% paint_simulator Marks map cells as painted if robot is within painting range
% paint: struct with width, spacing, etc.

% Identify world points within a small rectangle in front of robot representing spray coverage
sprayDist = paint.width; % approximate distance from wall
% sample several points around the robot near the wall side
numSamples = 30;
theta = pose(3);
% sample points in a semicircle in front of robot (assumes robot faces wall)
angles = linspace(-pi/3, pi/3, numSamples) + theta;
dists = linspace(0.1, sprayDist, numSamples);
for i=1:numSamples
    x = pose(1) + dists(i)*cos(angles(i));
    y = pose(2) + dists(i)*sin(angles(i));
    try
        [r,c] = world2grid(occMap, [x y]);
        r = round(r); c = round(c);
        if r>=1 && c>=1 && r<=size(paintMap,1) && c<=size(paintMap,2)
            % only mark if cell is near a wall (i.e., neighbor occupied)
            neigh = occMap.OccupancyMatrix(max(1,r-1):min(end,r+1), max(1,c-1):min(end,c+1));
            if any(neigh(:) > 0.5)
                paintMap(r,c) = true;
            end
        end
    catch
        % ignore
    end
end

end
