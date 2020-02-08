function [cords,loc] = label2cord(node_label,FEM_nodes)

% based on nodal label to return coordinates

% cords =[label, x,y,z]




total_nodes_number = size(FEM_nodes,1);


try
    for num=1:total_nodes_number
        
        
        if FEM_nodes(num,1) == node_label
            
            
            loc = num;
            
        end
    end
    
    
    cords = FEM_nodes(loc,:);
    
catch
    cords =nan;
end