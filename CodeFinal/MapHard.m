function [m, s, g] = MapHard %create_hard_map()
    m = zeros(30, 30);
    s = [2, 2]; 
    g = [28, 28]; 
    
    layers = [7, 13, 19, 25];
    gates = [9, 18, 27, 21];
    gate_width = 5;
    
    for i = 1:length(layers)
        row = layers(i);
        m(row, 1:30) = 1;
        
        g_start = gates(i);
        m(row, g_start : g_start + gate_width - 1) = 0;
    end
    
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end