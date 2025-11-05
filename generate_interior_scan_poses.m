function poses = generate_interior_scan_poses(poly, centroid)
    % generate a few interior poses for scanning: centroid plus some offsets
    % We ensure poses are inside polygon by taking midpoints of edges towards centroid
    poses = centroid; % start with centroid
    n = size(poly,1);
    for i=1:n
        a = poly(i,:);
        b = poly(mod(i,n)+1,:);
        mid = (a+b)/2;
        % move a bit towards centroid so pose is inside
        pose = mid*0.6 + centroid*0.4;
        % verify inside using inpolygon
        if inpolygon(pose(1), pose(2), poly(:,1), poly(:,2))
            poses = [poses; pose]; %#ok<AGROW>
        else
            % fallback: slightly towards centroid
            poses = [poses; (mid+centroid)/2]; %#ok<AGROW>
        end
    end
    % unique and small set
    poses = unique(poses,'rows','stable');
end
