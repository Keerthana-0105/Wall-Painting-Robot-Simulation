function y = project_point_from_line_param(L, t)
    a=L(1); b=L(2);
    dir = [-b, a]; dir = dir / norm(dir);
    y = t;
end