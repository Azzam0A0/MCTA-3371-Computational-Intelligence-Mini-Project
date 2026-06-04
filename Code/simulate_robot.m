function results = simulate_robot(map, start, goal, fis, showPlot)
    % SIMULATE_ROBOT Runs the navigation simulation
    
    maxSteps = 200;
    pos = start;
    dir = 2; % Start facing Right (1=Up, 2=Right, 3=Down, 4=Left)
    
    % FIX 1: PREALLOCATE path array to prevent lag and warnings
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
                
                % FIX 2: Prevent infinite collision spinning
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