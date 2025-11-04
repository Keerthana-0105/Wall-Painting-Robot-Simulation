function wallPath = plan_wall_following(boundary, spacing)
    % Interpolate points along the wall just inside
    offset = 0.1; % Offset inside the wall
    wallPath = [];
    for i = 1:(size(boundary,1)-1)
        p1 = boundary(i,:);
        p2 = boundary(i+1,:);
        dp = p2 - p1;
        L = norm(dp);
        nPts = ceil(L / spacing);
        for k = 0:nPts
            pt = p1 + k/nPts*dp;
            % Offset inward (normal)
            normal = dp / L;
            normal = [-normal(2), normal(1)]; % Rotate 90 deg
            pt_in = pt + offset * normal;
            wallPath = [wallPath; pt_in];
        end
    end
end
