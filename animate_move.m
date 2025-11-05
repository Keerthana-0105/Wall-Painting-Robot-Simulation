function animate_move(robot_h, start, goal, dt)
    steps = max(2, ceil(norm(goal-start)/0.01));
    for s=1:steps
        frac = s/steps;
        pos = start*(1-frac) + goal*frac;
        set(robot_h, 'XData', pos(1), 'YData', pos(2));
        drawnow limitrate;
    end
end