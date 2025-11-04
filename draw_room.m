function [boundary, occMap, wallSegments] = draw_room(mapResolution)
    figure('Name','Draw the Room Layout');
    axis([0 5 0 5]); grid on; hold on;
    title('Click to define the room boundary (clockwise or CCW). Press Enter when done.');
    [x, y] = getline; % User draws room by clicking
    boundary = [x, y];
    if size(boundary,1) < 3
        error('Room should have at least 3 points!');
    end
    % Close boundary
    if norm(boundary(1,:) - boundary(end,:)) > 1e-2
        boundary = [boundary; boundary(1,:)];
    end
    % Render walls
    wallSegments = [boundary(1:end-1,:) boundary(2:end,:)];
    for k = 1:size(wallSegments,1)
        plot(wallSegments(k,[1,3]), wallSegments(k,[2,4]), 'k-', 'LineWidth', 2);
    end

    % Create occupancy map
    occMap = occupancyMap(5, 5, mapResolution);
    polyin = polyshape(boundary(:,1), boundary(:,2));
    [X, Y] = meshgrid(linspace(0,5,5*mapResolution), linspace(0,5,5*mapResolution));
    in = isinterior(polyin,X(:),Y(:));
    occMap.setOccupancy([X(:),Y(:)], ~in); % Free inside, occupied outside
end
