function [x, y] = get_polygon_from_user()
    % get polygon vertices from user clicks until Enter is pressed
    pts = [];
    title('Click vertices, press Enter when finished.');
    but = 1;

    % Pre-set a fixed window area so MATLAB doesn’t zoom each time
    axis equal;
    xlim([-1 1]); ylim([-1 1]);  % Default visible area (you can adjust)
    hold on; grid on;

    first_click = true;

    while true
        [xi, yi, but] = ginput(1);
        if isempty(but) || but==13 || but==3  % Enter or right-click: finish
            break;
        end
        pts(end+1,:) = [xi yi]; %#ok<AGROW>

        % After first click, fix axis range so it stops resizing
        if first_click
            xlim manual; ylim manual;  % lock current limits
            first_click = false;
        end

        % Plot the clicked vertex and connecting edge
        plot(xi, yi, 'ro', 'MarkerFaceColor','r');
        if size(pts,1) > 1
            plot(pts(end-1:end,1), pts(end-1:end,2), '-k', 'LineWidth', 1.5);
        end
    end

    % Close polygon visually
    if ~isempty(pts)
        plot([pts(end,1) pts(1,1)], [pts(end,2) pts(1,2)], '-k','LineWidth',1.5);
    end

    % Output coordinates
    if isempty(pts)
        x = []; y = [];
    else
        x = pts(:,1); y = pts(:,2);
    end
end
