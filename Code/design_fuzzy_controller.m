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
    % Negative = turn left, Positive = turn right, 0 = straight
    fis = addOutput(fis, [-30 30], 'Name', 'Steering');
    fis = addMF(fis, 'Steering', 'constant', -20, 'Name', 'TurnLeft');
    fis = addMF(fis, 'Steering', 'constant', 0, 'Name', 'Straight');
    fis = addMF(fis, 'Steering', 'constant', 20, 'Name', 'TurnRight');

    % --- SIMPLE RULES (prioritize obstacle avoidance) ---
    % Format: [Front, Left, Right, Angle, Output, Weight, Connection]
    rules = [
        % If front is close, turn away from nearest obstacle
        1 1 2 1 3 1 1;  % Front close, Left close, Right far, Angle left -> Turn Right
        1 2 1 3 1 1 1;  % Front close, Left far, Right close, Angle right -> Turn Left
        1 2 2 2 1 1 1;  % Front close, Left far, Right far -> Turn Left (default)
        
        % If front is far, move toward goal
        2 1 1 1 1 1 1;  % Front far, both sides close, goal left -> Turn Left
        2 1 1 3 3 1 1;  % Front far, both sides close, goal right -> Turn Right
        2 2 2 2 2 1 1;  % Front far, sides far, goal straight -> Go Straight
        2 2 2 1 1 1 1;  % Front far, sides far, goal left -> Turn Left
        2 2 2 3 3 1 1;  % Front far, sides far, goal right -> Turn Right
    ];
    
    fis = addRule(fis, rules);
end
