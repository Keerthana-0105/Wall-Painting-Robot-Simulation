function y = project_point_from_line_param(L, t)
    % reconstruct point from parameter t along direction (used internally)
    a=L(1); b=L(2);
    dir = [-b, a]; dir = dir / norm(dir);
    y = t; % placeholder (we actually compute endpoints differently in main)
end