
function colors = lines_color_map(n)
    cmap = lines(max(3,n));
    colors = cmap(1:n,:);
end
