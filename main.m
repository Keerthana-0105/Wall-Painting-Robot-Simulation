close all; rng('shuffle');

figure('Name','Draw room layout','NumberTitle','off','Color',[1 1 1]);
axis equal; hold on; grid on;
title('Draw room layout');
xlabel('X (m)'); ylabel('Y (m)');

[x, y] = get_polygon_from_user();
poly = [x(:), y(:)];
if size(poly,1) < 3
    error('Need at least 3 vertices to form a room.');
end
plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], '-k','LineWidth',2);
fill(poly(:,1), poly(:,2), [0.95 0.95 1],'FaceAlpha',0.2,'EdgeColor','none');

lidar_range = 10;
num_beams = 720;
lidar_noise_std = 0.005;
robot_height = 0;
centroid = mean(poly);
scan_poses = generate_interior_scan_poses(poly, centroid);

lidar_points = [];
for i = 1:size(scan_poses,1)
    pose = scan_poses(i,:);
    pts = simulate_lidar_scan(poly, pose, num_beams, lidar_range, lidar_noise_std);
    lidar_points = [lidar_points; pts];
end

hpts = plot(lidar_points(:,1), lidar_points(:,2), '.','MarkerSize',6);
legend('room boundary','lidar points');

max_lines = 20;
inlier_threshold = 0.02;
min_inliers = 40;
[lines, line_inliers] = ransac_lines(lidar_points, max_lines, inlier_threshold, min_inliers);

colors = lines_color_map(length(lines));
wall_segments = cell(length(lines),1);
for i = 1:length(lines)
    L = lines{i};
    inliers = lidar_points(line_inliers{i},:);
    proj_coords = project_points_on_line(inliers, L);
    tmin = min(proj_coords(:,1)); tmax = max(proj_coords(:,1));
    p1 = [tmin, project_point_from_line_param(L,tmin)];
    p2 = [tmax, project_point_from_line_param(L,tmax)];
    dir = [ -L(2), L(1) ]; dir = dir / norm(dir);
    center_pt = mean(inliers,1);
    ep1 = center_pt + dir * (tmin - mean(proj_coords(:,1)));
    ep2 = center_pt + dir * (tmax - mean(proj_coords(:,1)));
    wall_segments{i} = [ep1; ep2];

    plot([ep1(1) ep2(1)], [ep1(2) ep2(2)], '-','LineWidth',3,'Color',colors(i,:));
end
title(sprintf('Detected %d walls (lines).', length(lines)));
fprintf("Number of walls detected: %d\n", length(lines));

figure('Name','Robot Moving Along Room Boundary','NumberTitle','off','Color',[1 1 1]);
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)');
title('Robot Following the Room Boundary');
fill(poly(:,1), poly(:,2), [0.9 0.95 1],'FaceAlpha',0.4,'EdgeColor','b','LineWidth',2);
plot(poly(:,1), poly(:,2), 'b-', 'LineWidth', 2);
plot(poly(:,1), poly(:,2), 'bo', 'MarkerFaceColor','b','DisplayName','Boundary Points');
legend('Room Boundary','Boundary Vertices','Location','bestoutside');

robot_radius = 0.03;
robot_color = [1 0.2 0.2];
robot_h = plot(poly(1,1), poly(1,2), 'o', 'MarkerFaceColor', robot_color,'MarkerEdgeColor','k', 'MarkerSize', 10, 'DisplayName','Robot');

num_vertices = size(poly,1);
pause(1);
for i = 1:num_vertices
    start_pt = poly(i,:);
    if i < num_vertices
        end_pt = poly(i+1,:);
    else
        end_pt = poly(1,:);
    end
    steps = max(2, ceil(norm(end_pt - start_pt)/0.01));
    for s = 1:steps
        frac = s / steps;
        pos = (1 - frac) * start_pt + frac * end_pt;
        set(robot_h, 'XData', pos(1), 'YData', pos(2));
        drawnow limitrate;
        pause(0.01);
    end
    plot([start_pt(1) end_pt(1)], [start_pt(2) end_pt(2)], '-', 'LineWidth', 3, 'Color', [0 0.8 0], 'DisplayName','Painted Wall');
    drawnow;
    fprintf("Painted wall %d (task %d of %d)\n", i, i, num_vertices);
end

fprintf("Painting finished. Total walls painted: %d\n", num_vertices);
