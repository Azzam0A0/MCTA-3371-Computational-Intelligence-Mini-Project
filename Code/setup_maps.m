function [map, start, goal] = setup_maps(mapType)
    % SETUP_MAPS Generates 2D grid maps for robot navigation
    % mapType: 'simple' or 'complex'
    
    if strcmp(mapType, 'simple')
        map = zeros(20, 20);
        % Add a few simple block obstacles
        map(5:8, 8:12) = 1;
        map(12:15, 4:7) = 1;
        start = [2, 2];
        goal = [19, 19];
    elseif strcmp(mapType, 'complex')
        map = zeros(20, 20);
        % Maze-like / Cluttered layout
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