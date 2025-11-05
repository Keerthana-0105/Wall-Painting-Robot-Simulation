
function pts = simulate_lidar_scan(poly, pose, num_beams, max_range, noise_std)
    angles = linspace(-pi, pi, num_beams);
    rays = [cos(angles(:)), sin(angles(:))];
    edges = [poly; poly(1,:)];
    pts = zeros(num_beams,2);
    for i = 1:num_beams
        p0 = pose(:)';
        dir = rays(i,:);
        min_dist = max_range;
        hit = p0 + dir * max_range;
        for e = 1:size(poly,1)
            A = edges(e,:);
            B = edges(e+1,:);
            [intersect, P] = ray_segment_intersection(p0, dir, A, B);
            if intersect
                d = norm(P - p0);
                if d < min_dist
                    min_dist = d;
                    hit = P;
                end
            end
        end
        dnoisy = min_dist + randn()*noise_std;
        dnoisy = max(0, min(dnoisy, max_range));
        pts(i,:) = p0 + dir * dnoisy;
    end
    valid = vecnorm(pts - pose, 2, 2) < (max_range - 1e-6);
    pts = pts(valid,:);
end