function [intersect, P] = ray_segment_intersection(p0, d, A, B)
    % p0 + t*d intersects segment AB?
    % Solve p0 + t*d = A + u*(B-A), with t>=0, u in [0,1]
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
