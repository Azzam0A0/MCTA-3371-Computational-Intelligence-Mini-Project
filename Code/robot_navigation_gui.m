function robot_navigation_gui(map, start, goal, fis)
    fig = uifigure('Name', 'Intelligent Robot Navigation GUI', 'Position', [100, 100, 800, 500]);
    
    ax = uiaxes(fig, 'Position', [20, 50, 400, 400]);
    title(ax, 'Navigation Environment');
    
    uilabel(fig, 'Text', 'Control Panel', 'FontWeight', 'bold', 'Position', [450, 400, 150, 22]);
    
    lblMetrics = uilabel(fig, 'Text', 'Metrics will appear here...', 'Position', [450, 250, 300, 80], 'WordWrap', 'on');
    
    btnSim = uibutton(fig, 'Text', 'Run Simulation', 'Position', [450, 350, 150, 30]);
    
    % FIX: Store all needed variables in the button's UserData
    btnSim.UserData = struct('ax', ax, 'map', map, 'start', start, 'goal', goal, 'fis', fis, 'lblMetrics', lblMetrics);
    
    % Assign callback cleanly
    btnSim.ButtonPushedFcn = @run_sim_callback;
    
    plot_map_step(map, start, goal, start, start, ax);
end

function run_sim_callback(btn, ~)
    % Retrieve data safely from UserData
    data = btn.UserData;
    
    % Run logic
    results = simulate_robot(data.map, data.start, data.goal, data.fis, false);
    
    % Animate path
    plot_map_step(data.map, data.start, data.goal, results.path, results.path(end,:), data.ax);
    
    % Update Metrics
    status = 'Success';
    if ~results.success
        status = 'Failed';
    end
    
    txt = sprintf('Status: %s\nSteps Taken: %d\nCollisions: %d\nPath Length: %.2f', ...
        status, results.steps, results.collisions, results.pathLength);
    data.lblMetrics.Text = txt;
end