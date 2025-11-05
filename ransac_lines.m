function [lines, inlier_sets] = ransac_lines(points, max_lines, thresh, min_inliers)
    remaining = points;
    lines = {}; inlier_sets = {};
    iters = 1200;
    while size(remaining,1) >= min_inliers && length(lines) < max_lines
        best_count = 0; best_line = []; best_inliers = [];
        N = size(remaining,1);
        for it = 1:iters
            idx = randperm(N,2);
            p1 = remaining(idx(1),:); p2 = remaining(idx(2),:);
            if norm(p1-p2) < 1e-3, continue; end
            a = p2(2)-p1(2); b = p1(1)-p2(1);
            norm_ab = sqrt(a^2 + b^2);
            a = a / norm_ab; b = b / norm_ab;
            c = -(a*p1(1) + b*p1(2));
            dists = abs(a*remaining(:,1) + b*remaining(:,2) + c);
            inliers = find(dists <= thresh);
            if numel(inliers) > best_count
                best_count = numel(inliers);
                best_line = [a b c];
                best_inliers = inliers;
            end
        end
        if best_count >= min_inliers
            lines{end+1} = best_line;
            inlier_sets{end+1} = find(abs(best_line(1)*remaining(:,1) + best_line(2)*remaining(:,2) + best_line(3)) <= thresh); %#ok<AGROW>
            mask = true(size(remaining,1),1);
            mask(inlier_sets{end}) = false;
            remaining = remaining(mask,:);
        else
            break;
        end
    end

    for i=1:length(lines)
        inpts = points(inlier_sets{i},:);
        if size(inpts,1) >= 2
            mu = mean(inpts,1);
            C = (inpts - mu)' * (inpts - mu);
            [V,D] = eig(C);
            [~, idx] = max(diag(D));
            dir = V(:,idx)';
            a = -dir(2); b = dir(1);
            norm_ab = sqrt(a^2 + b^2);
            a = a / norm_ab; b = b / norm_ab;
            c = -(a*mu(1) + b*mu(2));
            lines{i} = [a b c];
        end
    end
end
