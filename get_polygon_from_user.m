function [x, y] = get_polygon_from_user()
    pts = [];
    title('Click vertices.');
    but = 1;

    axis equal;
    xlim([-1 1]); ylim([-1 1]);
    hold on; grid on;

    first_click = true;

    while true
        [xi, yi, but] = ginput(1);
        if isempty(but) || but==13 || but==3
            break;
        end
        pts(end+1,:) = [xi yi];

        if first_click
            xlim manual; ylim manual;
            first_click = false;
        end

        plot(xi, yi, 'ro', 'MarkerFaceColor','r');
        if size(pts,1) > 1
            plot(pts(end-1:end,1), pts(end-1:end,2), '-k', 'LineWidth', 1.5);
        end
    end

    if ~isempty(pts)
        plot([pts(end,1) pts(1,1)], [pts(end,2) pts(1,2)], '-k','LineWidth',1.5);
    end

    if isempty(pts)
        x = []; y = [];
    else
        x = pts(:,1); y = pts(:,2);
    end
end
