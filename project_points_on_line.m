function proj = project_points_on_line(pts, L)
    a=L(1); b=L(2); c=L(3);
    dir = [-b, a];
    dir = dir / norm(dir);
    origin = mean(pts,1);
    proj_t = (pts - origin) * dir';
    proj = [proj_t, zeros(size(proj_t))];
end