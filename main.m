% wall_painting_slam.m
% Interactive room drawing -> LIDAR simulation -> RANSAC wall detection ->
% simple painting path planning & simulation
%
% Usage: run the file. Click polygon vertices in the figure window to draw
% the room boundary. Press Enter/Return when done. The simulation runs
% automatically and prints number of walls painted.

close all; rng('shuffle');

% ---------- Step 1: User draws room boundary ----------
figure('Name','Draw room boundary: Click vertices, press Enter when finished',...
       'NumberTitle','off','Color',[1 1 1]);
axis equal; hold on; grid on;
title('Click vertices around the room boundary (clockwise or anticlockwise). Press Enter when done.');
xlabel('X (m)'); ylabel('Y (m)');
% Let user click polygon vertices
[x, y] = get_polygon_from_user();
poly = [x(:), y(:)];
if size(poly,1) < 3
    error('Need at least 3 vertices to form a room.');
end
plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], '-k','LineWidth',2);
fill(poly(:,1), poly(:,2), [0.95 0.95 1],'FaceAlpha',0.2,'EdgeColor','none');

% ---------- Step 2: Set up simulation parameters ----------
lidar_range = 10;          % max LIDAR range (m)
num_beams = 720;          % beams per scan (360 deg)
lidar_noise_std = 0.005;  % gaussian noise on ranges (m)
robot_height = 0;         % unused; 2D sim
% choose scanning poses: center + a grid of interior points for better coverage
centroid = mean(poly);
scan_poses = generate_interior_scan_poses(poly, centroid);

% simulate aggregate point cloud from multiple scans
lidar_points = [];
for i = 1:size(scan_poses,1)
    pose = scan_poses(i,:);
    pts = simulate_lidar_scan(poly, pose, num_beams, lidar_range, lidar_noise_std);
    lidar_points = [lidar_points; pts]; %#ok<AGROW>
end

% plot raw lidar points
hpts = plot(lidar_points(:,1), lidar_points(:,2), '.','MarkerSize',6);
legend('room boundary','lidar points');

% ---------- Step 3: Detect lines (walls) with simple RANSAC ----------
max_lines = 20;
inlier_threshold = 0.02;   % meters
min_inliers = 40;          % minimum points to accept a line
[lines, line_inliers] = ransac_lines(lidar_points, max_lines, inlier_threshold, min_inliers);

% plot detected lines
colors = lines_color_map(length(lines));
wall_segments = cell(length(lines),1);
for i = 1:length(lines)
    L = lines{i}; % [a b c] line ax + by + c = 0 normalized
    inliers = lidar_points(line_inliers{i},:);
    % compute endpoints of segment by projecting inliers onto the line and taking min/max
    proj_coords = project_points_on_line(inliers, L);
    tmin = min(proj_coords(:,1)); tmax = max(proj_coords(:,1));
    p1 = [tmin, project_point_from_line_param(L,tmin)];
    p2 = [tmax, project_point_from_line_param(L,tmax)];
    % but easier: compute endpoints using 2D line paramization via direction vector
    dir = [ -L(2), L(1) ]; dir = dir / norm(dir);
    center_pt = mean(inliers,1);
    ep1 = center_pt + dir * (tmin - mean(proj_coords(:,1)));
    ep2 = center_pt + dir * (tmax - mean(proj_coords(:,1)));
    wall_segments{i} = [ep1; ep2];

    plot([ep1(1) ep2(1)], [ep1(2) ep2(2)], '-','LineWidth',3,'Color',colors(i,:));
end
title(sprintf('Detected %d walls (lines).', length(lines)));
fprintf("Number of walls detected: %d\n", length(lines));

% ---------- Step 4: Create Figure 2 for Robot Boundary Movement ----------
figure('Name','Robot Moving Along Room Boundary','NumberTitle','off','Color',[1 1 1]);
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)');
title('Robot Following the Room Boundary');
fill(poly(:,1), poly(:,2), [0.9 0.95 1],'FaceAlpha',0.4,'EdgeColor','b','LineWidth',2);
plot(poly(:,1), poly(:,2), 'b-', 'LineWidth', 2);
plot(poly(:,1), poly(:,2), 'bo', 'MarkerFaceColor','b','DisplayName','Boundary Points');
legend('Room Boundary','Boundary Vertices','Location','bestoutside');

% Robot settings
robot_radius = 0.03;
robot_color = [1 0.2 0.2];
robot_h = plot(poly(1,1), poly(1,2), 'o', 'MarkerFaceColor', robot_color, ...
               'MarkerEdgeColor','k', 'MarkerSize', 10, 'DisplayName','Robot');

% Animate robot along boundary
num_vertices = size(poly,1);
pause(1);
for i = 1:num_vertices
    start_pt = poly(i,:);
    if i < num_vertices
        end_pt = poly(i+1,:);
    else
        end_pt = poly(1,:); % close the loop
    end
    steps = max(2, ceil(norm(end_pt - start_pt)/0.01));
    for s = 1:steps
        frac = s / steps;
        pos = (1 - frac) * start_pt + frac * end_pt;
        set(robot_h, 'XData', pos(1), 'YData', pos(2));
        drawnow limitrate;
        pause(0.01);
    end
    % Mark painted wall visually
    plot([start_pt(1) end_pt(1)], [start_pt(2) end_pt(2)], '-', ...
        'LineWidth', 3, 'Color', [0 0.8 0], 'DisplayName','Painted Wall');
    drawnow;
    fprintf("Painted wall %d (task %d of %d)\n", i, i, num_vertices);
end

fprintf("Painting finished. Total walls painted: %d\n", num_vertices);
