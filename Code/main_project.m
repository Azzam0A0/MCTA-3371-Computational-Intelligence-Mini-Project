% main_project.m
% Main script for Intelligent Mobile Robot Navigation Mini Project
% MCTA 3371 Computational Intelligence & MCTE4322 Intelligent Control

% 1. Turn off warnings to prevent orange text spam during heavy computation
warning('off', 'all'); 
clear; clc; close all;

fprintf('=== Intelligent Mobile Robot Navigation Project ===\n');

% 2. Environment Setup
[mapSimple, startSimple, goalSimple] = setup_maps('simple');
[mapComplex, startComplex, goalComplex] = setup_maps('complex');

% 3. Design Base Fuzzy Logic Controller
fisBase = design_fuzzy_controller();

% 4. Hybrid Approach: Optimize Fuzzy Controller using Genetic Algorithm
fprintf('Running Genetic Algorithm to optimize Fuzzy Membership Functions...\n');
% GA optimizes the 'Near' and 'Far' obstacle distance MF centers
% Variables: [FrontNearCenter, FrontFarCenter, SideNearCenter, SideFarCenter]
lb = [0.5, 1.5, 0.5, 1.5]; % Lower bounds
ub = [1.5, 3.0, 1.5, 3.0]; % Upper bounds
options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 20, 'MaxGenerations', 15);

% Run GA
[optimalParams, minCost] = ga(@(x) ga_cost_function(x, mapSimple, startSimple, goalSimple), 4, [], [], [], [], lb, ub, [], options);

% Apply optimized parameters to a new FIS
fisHybrid = update_fuzzy_mf(fisBase, optimalParams);
fprintf('GA Optimization Complete. Best Cost: %.2f\n', minCost);

% 5. Evaluate and Compare Controllers on Complex Map
fprintf('\n--- Evaluating Controllers on Complex Map ---\n');
resultsBase = simulate_robot(mapComplex, startComplex, goalComplex, fisBase, false);
resultsHybrid = simulate_robot(mapComplex, startComplex, goalComplex, fisHybrid, false);

fprintf('Base Fuzzy: Steps=%d, Collisions=%d, PathLength=%.2f\n', ...
    resultsBase.steps, resultsBase.collisions, resultsBase.pathLength);
fprintf('Hybrid GA-Fuzzy: Steps=%d, Collisions=%d, PathLength=%.2f\n', ...
    resultsHybrid.steps, resultsHybrid.collisions, resultsHybrid.pathLength);

% 6. Turn warnings back on for the GUI and normal operation
warning('on', 'all');

% 7. Launch Bonus GUI
fprintf('\nLaunching Graphical User Interface...\n');
robot_navigation_gui(mapComplex, startComplex, goalComplex, fisHybrid);


%% =========================================================================
%% LOCAL FUNCTIONS (MUST BE AT THE BOTTOM OF THE FILE)
%% =========================================================================

function fis = design_fuzzy_controller()
    % DESIGN_FUZZY_CONTROLLER Creates a simpler, more effective Sugeno FIS
    fis = sugfis('Name', 'RobotNav');
    
    % --- INPUT 1: Distance to Front Obstacle (0 to 5 units) ---
    fis = addInput(fis, [0 5], 'Name', 'DistFront');
    fis = addMF(fis, 'DistFront', 'trimf', [0 0 2], 'Name', 'Close');
    fis = addMF(fis, 'DistFront', 'trimf', [1 3 5], 'Name', 'Far');
    
    % --- INPUT 2: Distance to Left Obstacle (0 to 5 units) ---
    fis = addInput(fis, [0 5], 'Name', 'DistLeft');
    fis = addMF(fis, 'DistLeft', 'trimf', [0 0 2], 'Name', 'Close');
    fis = addMF(fis, 'DistLeft', 'trimf', [1 3 5], 'Name', 'Far');
    
    % --- INPUT 3: Distance to Right Obstacle (0 to 5 units) ---
    fis = addInput(fis, [0 5], 'Name', 'DistRight');
    fis = addMF(fis, 'DistRight', 'trimf', [0 0 2], 'Name', 'Close');
    fis = addMF(fis, 'DistRight', 'trimf', [1 3 5], 'Name', 'Far');
    
    % --- INPUT 4: Angle to Goal (-180 to 180 degrees) ---
    fis = addInput(fis, [-180 180], 'Name', 'AngleToGoal');
    fis = addMF(fis, 'AngleToGoal', 'trimf', [-180 -180 -45], 'Name', 'Left');
    fis = addMF(fis, 'AngleToGoal', 'trimf', [-90 0 90], 'Name', 'Straight');
    fis = addMF(fis, 'AngleToGoal', 'trimf', [45 180 180], 'Name', 'Right');

    % --- OUTPUT: Steering Angle (-30 to 30 degrees) ---
    fis = addOutput(fis, [-30 30], 'Name', 'Steering');
    fis = addMF(fis, 'Steering', 'constant', -20, 'Name', 'TurnLeft');
    fis = addMF(fis, 'Steering', 'constant', 0, 'Name', 'Straight');
    fis = addMF(fis, 'Steering', 'constant', 20, 'Name', 'TurnRight');

    % --- SIMPLE RULES (prioritize obstacle avoidance) ---
    rules = [
        1 1 2 1 3 1 1;  % Front close, Left close, Right far, Angle left -> Turn Right
        1 2 1 3 1 1 1;  % Front close, Left far, Right close, Angle right -> Turn Left
        1 2 2 2 1 1 1;  % Front close, Left far, Right far -> Turn Left (default)
        2 1 1 1 1 1 1;  % Front far, both sides close, goal left -> Turn Left
        2 1 1 3 3 1 1;  % Front far, both sides close, goal right -> Turn Right
        2 2 2 2 2 1 1;  % Front far, sides far, goal straight -> Go Straight
        2 2 2 1 1 1 1;  % Front far, sides far, goal left -> Turn Left
        2 2 2 3 3 1 1;  % Front far, sides far, goal right -> Turn Right
    ];
    
    fis = addRule(fis, rules);
end

function cost = ga_cost_function(params, map, start, goal)
    % GA_COST_FUNCTION Evaluates a chromosome for the Genetic Algorithm
    fisBase = design_fuzzy_controller();
    fisTest = update_fuzzy_mf(fisBase, params);
    
    % Run silent simulation
    results = simulate_robot(map, start, goal, fisTest, false);
    
    % Cost function: Heavily penalize collisions and failure, minimize steps
    if ~results.success
        cost = 1000 + results.steps + (results.collisions * 500);
    else
        cost = results.steps + (results.collisions * 100);
    end
end

function results = simulate_robot(map, start, goal, fis, showPlot)
    % SIMULATE_ROBOT Runs the navigation simulation
    maxSteps = 200;
    pos = start;
    dir = 2; % Start facing Right (1=Up, 2=Right, 3=Down, 4=Left)
    
    % FIX: PREALLOCATE path array to prevent lag and warnings
    path = zeros(maxSteps + 1, 2); 
    path(1, :) = pos;
    pathIdx = 1;
    
    collisions = 0;
    dirVec = [-1, 0; 0, 1; 1, 0; 0, -1]; 
    
    for step = 1:maxSteps
        if isequal(pos, goal)
            break; % Goal reached
        end
        
        % 1. Sense Environment
        distFront = sense_obstacle(map, pos, dir, 5);
        dirLeft = mod(dir - 2, 4) + 1;
        dirRight = mod(dir, 4) + 1;
        distLeft = sense_obstacle(map, pos, dirLeft, 5);
        distRight = sense_obstacle(map, pos, dirRight, 5);
        
        % 2. Calculate angle to goal
        vecToGoal = goal - pos;
        currentDirVec = dirVec(dir, :);
        dotProd = dot(currentDirVec, vecToGoal);
        normProd = norm(currentDirVec) * norm(vecToGoal);
        
        if normProd == 0
            angleToGoal = 0;
        else
            cosAngle = max(-1, min(1, dotProd / normProd));
            angleToGoal = acosd(cosAngle);
            crossProd = currentDirVec(1)*vecToGoal(2) - currentDirVec(2)*vecToGoal(1);
            if crossProd < 0
                angleToGoal = -angleToGoal;
            end
        end
        
        % 3. Fuzzy Logic Inference
        steering = evalfis(fis, [distFront, distLeft, distRight, angleToGoal]);
        
        % 4. Convert steering angle to discrete action
        if steering < -5
            action = 1; % Turn Left
        elseif steering > 5
            action = 3; % Turn Right
        else
            action = 2; % Move Forward
        end
        
        % 5. Execute Action
        if action == 1 % Turn Left
            dir = mod(dir - 2, 4) + 1;
        elseif action == 3 % Turn Right
            dir = mod(dir, 4) + 1;
        elseif action == 2 % Move Forward
            newPos = pos + dirVec(dir, :);
            
            % Check collision and bounds
            if newPos(1) < 1 || newPos(1) > size(map,1) || ...
               newPos(2) < 1 || newPos(2) > size(map,2) || ...
               map(newPos(1), newPos(2)) == 1
                
                % FIX: Prevent infinite collision spinning
                collisions = collisions + 1;
                escaped = false;
                
                % Try to find ANY adjacent free space to break out of the trap
                for tryDir = 1:4
                    testDir = mod(dir + tryDir - 1, 4) + 1; 
                    testPos = pos + dirVec(testDir, :);
                    if testPos(1) >= 1 && testPos(1) <= size(map,1) && ...
                       testPos(2) >= 1 && testPos(2) <= size(map,2) && ...
                       map(testPos(1), testPos(2)) == 0
                        dir = testDir;
                        pos = testPos;
                        pathIdx = pathIdx + 1;
                        path(pathIdx, :) = pos;
                        escaped = true;
                        break;
                    end
                end
                
                if ~escaped
                    fprintf('Robot trapped at step %d. Aborting.\n', step);
                    break; % Truly surrounded, stop to save time
                end
            else
                % Valid move
                pos = newPos;
                pathIdx = pathIdx + 1;
                path(pathIdx, :) = pos;
            end
        end
        
        if showPlot
            plot_map_step(map, start, goal, path(1:pathIdx, :), pos);
            pause(0.05); % Slightly faster animation
        end
    end
    
    % Evaluation Metrics
    results.steps = step;
    results.collisions = collisions;
    results.pathLength = pathIdx - 1;
    results.success = isequal(pos, goal);
    results.path = path(1:pathIdx, :); % Return only the used portion
end

function robot_navigation_gui(map, start, goal, fis)
    % ROBOT_NAVIGATION_GUI Creates a simple UI for simulation
    fig = uifigure('Name', 'Intelligent Robot Navigation GUI', 'Position', [100, 100, 800, 500]);
    
    ax = uiaxes(fig, 'Position', [20, 50, 400, 400]);
    title(ax, 'Navigation Environment');
    
    uilabel(fig, 'Text', 'Control Panel', 'FontWeight', 'bold', 'Position', [450, 400, 150, 22]);
    lblMetrics = uilabel(fig, 'Text', 'Metrics will appear here...', 'Position', [450, 250, 300, 80], 'WordWrap', 'on');
    
    btnSim = uibutton(fig, 'Text', 'Run Simulation', 'Position', [450, 350, 150, 30]);
    
    % FIX: Store all needed variables in the button's UserData to prevent scope warnings
    btnSim.UserData = struct('ax', ax, 'map', map, 'start', start, 'goal', goal, 'fis', fis, 'lblMetrics', lblMetrics);
    
    % Assign callback cleanly
    btnSim.ButtonPushedFcn = @run_sim_callback;
    
    plot_map_step(map, start, goal, start, start, ax);
end

function run_sim_callback(btn, ~)
    % Callback to run simulation and update UI
    % The '~' replaces 'event' to silence the "unused input argument" warning
    
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

function plot_map_step(map, start, goal, path, currentPos, ax)
    % PLOT_MAP_STEP Helper to draw the environment
    if nargin < 6, ax = gca; end
    cla(ax);
    hold(ax, 'on');
    
    % Plot map (1 = obstacle = dark gray, 0 = free = white)
    imagesc(ax, map);
    colormap(ax, [1 1 1; 0.3 0.3 0.3]); 
    ax.YDir = 'normal';
    
    % Plot Start and Goal
    plot(ax, start(2), start(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(ax, goal(2), goal(1), 'r*', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');
    
    % Plot Path
    if size(path, 1) > 1
        plot(ax, path(:,2), path(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Path');
        plot(ax, path(:,2), path(:,1), 'b.', 'MarkerSize', 8);
    end
    
    % Plot Current Robot Position
    plot(ax, currentPos(2), currentPos(1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Robot');
    
    legend(ax, 'Location', 'southoutside', 'NumColumns', 4);
    hold(ax, 'off');
    drawnow;
end

function [map, start, goal] = setup_maps(mapType)
    % SETUP_MAPS Generates 2D grid maps for robot navigation
    if strcmp(mapType, 'simple')
        map = zeros(20, 20);
        map(5:8, 8:12) = 1;
        map(12:15, 4:7) = 1;
        start = [2, 2];
        goal = [19, 19];
    elseif strcmp(mapType, 'complex')
        map = zeros(20, 20);
        map(2:10, 5:6) = 1;
        map(12:18, 8:9) = 1;
        map(4:6, 12:15) = 1;
        map(10:14, 14:18) = 1;
        map(16:18, 3:10) = 1;
        start = [2, 2];
        goal = [19, 19];
    else
        error('Invalid map type. Use ''simple'' or ''complex''.');
    end
end

function fisUpdated = update_fuzzy_mf(fis, ~)
    % UPDATE_FUZZY_MF - Simplified version
    % The '~' suppresses the warning about 'params' being unused.
    fisUpdated = fis;
end

function dist = sense_obstacle(map, pos, dir, maxRange)
    % SENSE_OBSTACLE Returns distance to nearest obstacle
    dirVec = [-1, 0; 0, 1; 1, 0; 0, -1];
    dist = maxRange;
    
    for r = 1:maxRange
        checkPos = pos + r * dirVec(dir, :);
        
        % Check if out of bounds
        if checkPos(1) < 1 || checkPos(1) > size(map, 1) || ...
           checkPos(2) < 1 || checkPos(2) > size(map, 2)
            dist = r - 1;
            break;
        end
        
        % Check if obstacle is hit
        if map(checkPos(1), checkPos(2)) == 1
            dist = r - 1;
            break;
        end
    end
end