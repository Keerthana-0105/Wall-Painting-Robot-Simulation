function poses = generate_interior_scan_poses(poly, centroid)
    poses = centroid;
    n = size(poly,1);
    for i=1:n
        a = poly(i,:);
        b = poly(mod(i,n)+1,:);
        mid = (a+b)/2;
        pose = mid*0.6 + centroid*0.4;
        if inpolygon(pose(1), pose(2), poly(:,1), poly(:,2))
            poses = [poses; pose];
        else
            poses = [poses; (mid+centroid)/2];
        end
    end
    poses = unique(poses,'rows','stable');
end
