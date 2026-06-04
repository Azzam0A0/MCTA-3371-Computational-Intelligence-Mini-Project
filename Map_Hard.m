function [m, s, g] = Map_Hard %create_hard_map()
    m = zeros(30, 30);
    s = [2, 15];
    g = [28, 15];
    
    layers = [7, 13, 19, 25];
    
    gates = [5, 25, 6, 24]; 
    gate_width = 2;
    
    for i = 1:length(layers)
        row = layers(i);
        m(row, 1:30) = 1;
        
        g_start = gates(i);
        m(row, g_start : g_start + gate_width - 1) = 0;
    end
    
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end

function [m, s, g] = create_hard_map()
    m = zeros(30, 30);
    s = [2, 15];
    g = [28, 15];
    
    layers = [7, 13, 19, 25];
    
    gates = [4, 8, 5, 9]; 
    gate_width = 2;
    
    for i = 1:length(layers)
        row = layers(i);
        m(row, 1:30) = 1;
        
        g_start = gates(i);
        m(row, g_start : g_start + gate_width - 1) = 0;
    end
    
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end

function [m, s, g] = create_hard_map()
    m = zeros(30, 30);
    
    s = [2, 2];   
    g = [28, 28];  
    
    layers = [7, 13, 19, 25];
    
    gates = [6, 14, 22, 8]; 
    gate_width = 2; 
    
    for i = 1:length(layers)
        row = layers(i);
        m(row, 1:30) = 1;
        
        g_start = gates(i);
        m(row, g_start : g_start + gate_width - 1) = 0;
    end
    
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end