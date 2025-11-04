function [isUpdated, pose, slamMap] = slam_update(slamObj, scan)
    isUpdated = false; pose = []; slamMap = [];

    try
        isAccepted = addScan(slamObj, scan);
        isUpdated = isAccepted;

        if slamObj.PoseGraph.NumNodes > 0
            poseGraphNodes = nodeEstimates(slamObj.PoseGraph);
            pose = poseGraphNodes(:, end); % 3 x 1 pose vector
        end

        try
            slamMap = slamObj.Map; % only if your MATLAB supports this property
        catch
            slamMap = [];
        end
    catch
        warning('SLAM update failed.');
    end
end
