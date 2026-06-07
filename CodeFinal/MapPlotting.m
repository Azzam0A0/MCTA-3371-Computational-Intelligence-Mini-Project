clear; clc; close all;

mapFunctions = {@MapEasy, @MapHard}; 
mapNames     = {'Easy Map Layout', 'Hard Map Layout'};

hFig = figure('Name', 'FYP Saved Maps Layout Comparison (imagesc)', ...
              'Color', 'w', ...
              'Position', [100, 200, 1100, 500]);

for mIdx = 1:length(mapFunctions)
    
    [map_grid, start_pos, goal_pos] = mapFunctions{mIdx}(); 
    
    [rows, cols] = size(map_grid);
    mapStruct.name = mapNames{mIdx};
    mapStruct.grid = map_grid;
    
    mapStruct.start = [start_pos(2), rows - start_pos(1) + 1];
    mapStruct.goal  = [goal_pos(2),  rows - goal_pos(1) + 1];
    
    PlotToSubplotImagesc(mapStruct, 1, 2, mIdx);
end

sgtitle('Robotic Path Planning - Environment Models Layout', ...
        'FontSize', 16, 'FontWeight', 'bold');

function PlotToSubplotImagesc(map, numRows, numCols, plotIdx)
    ax = subplot(numRows, numCols, plotIdx);
    
    cmap = [
        1.0  1.0  1.0;
        0.2  0.2  0.2;
        1.0  1.0  1.0;
        1.0  1.0  1.0
    ];
    
    imagesc(ax, flipud(map.grid)); 
    
    colormap(ax, cmap);
    clim(ax, [0 3]);
    
    set(ax, 'YDir', 'normal'); 
    axis(ax, 'equal', 'tight'); 
    grid(ax, 'on');
    hold(ax, 'on');
    
    hStart = plot(ax, map.start(1), map.start(2), 'go', 'MarkerSize', 11, ...
                  'LineWidth', 2, 'MarkerFaceColor', [0.4 0.9 0.4]);
    hGoal  = plot(ax, map.goal(1), map.goal(2), 'ro', 'MarkerSize', 11, ...
                  'LineWidth', 2, 'MarkerFaceColor', [1 0.4 0.4]);
    
    title(ax, map.name, 'FontSize', 13, 'FontWeight', 'bold');
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');
    legend(ax, [hStart, hGoal], {'Start','Goal'}, 'Location', 'best');
    hold(ax, 'off');
end