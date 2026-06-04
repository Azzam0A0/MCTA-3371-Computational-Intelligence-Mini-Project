function [m, s, g] = Map_Easy %create_easy_map()
    m = zeros(30, 30);
    s = [2, 5];
    g = [28, 15];
    
    obs = [
        10, 4,  8, 4;
        14, 3,  5, 6;
        
        2,  15, 9, 4; 
        5,  13, 5, 7;
        
        16, 21, 9, 4;
        18, 19, 5, 7;
        
        23, 11, 4, 4
    ]; 
    
    for i = 1:size(obs, 1)
        r_start = max(1, obs(i,2));
        r_end   = min(30, obs(i,2) + obs(i,4) - 1);
        c_start = max(1, obs(i,1));
        c_end   = min(30, obs(i,1) + obs(i,3) - 1);
        
        m(r_start:r_end, c_start:c_end) = 1;
    end
    
    m(s(1), s(2)) = 2; 
    m(g(1), g(2)) = 3;
end