function [v, w, state] = diffdrive_controller(pose, goal, robot, state)
% diffdrive_controller Simple controller to go to point
% pose: [x;y;theta], goal: [x;y]
% returns linear v and angular w, and state for integrators

if nargin < 4 || isempty(state)
    state = struct();
end

% Parameters
k_rho = 0.9;       % linear gain
k_alpha = 2.5;     % angular gain
k_beta = -0.6;     % final orientation gain (not used)

dx = goal(1) - pose(1);
dy = goal(2) - pose(2);
rho = hypot(dx, dy);
alpha = wrapToPi(atan2(dy,dx) - pose(3));
beta = wrapToPi(-pose(3) - alpha);

% Control law (unicycle)
v = k_rho * rho;
w = k_alpha * alpha + k_beta * beta;

% slow down for large heading error
if abs(alpha) > pi/4
    v = 0.12; % small forward while turning
end

% small thresholds
if rho < 0.05
    v = 0;
    w = 0;
end

end
