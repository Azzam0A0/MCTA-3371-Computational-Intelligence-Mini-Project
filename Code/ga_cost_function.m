function cost = ga_cost_function(params, map, start, goal)
    % GA_COST_FUNCTION Evaluates a chromosome for the Genetic Algorithm
    
    % Rebuild FIS with new parameters
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