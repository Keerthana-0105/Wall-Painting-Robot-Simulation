function [intersect, P] = ray_segment_intersection(p0, d, A, B)
    intersect = false; P=[0 0];
    v = B - A;
    M = [d(:), -v(:)];
    if abs(det(M)) < 1e-9
        return;
    end
    sol = M \ ( (A(:) - p0(:)) );
    t = sol(1); u = sol(2);
    if t >= 0 && u >= 0 && u <= 1
        intersect = true;
        P = (p0 + t*d);
    end
end
