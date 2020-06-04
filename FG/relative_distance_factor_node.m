classdef relative_distance_factor_node < factor_node
    %RELATIVE_DISTANCE_FACTOR_NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensor_pos_x
        sensor_pos_y
        type
    end
    
    properties (Constant)
       % define possible state
       type_A = 0;      % A_theta node
       type_B = 1;      % B_theta node
    end
   
    methods
        function dist_FN = relative_distance_factor_node(id,sensor_pos_x,sensor_pos_y,type)
            %RELATIVE_DISTANCE_FACTOR_NODE Construct an instance of this class
            %   Detailed explanation goes here
            dist_FN = dist_FN@factor_node(id);
            dist_FN.sensor_pos_x = sensor_pos_x;
            dist_FN.sensor_pos_y = sensor_pos_y;
            if type == 'A'
                dist_FN.type = dist_FN.type_A;
            else
                dist_FN.type = dist_FN.type_B;
            end
        end

        function reset(dist_FN)
           reset@factor_node(dist_FN);
        end
        
        function setup_link(dist_FN,linklist, parent_num, child_num)
           setup_link@factor_node(dist_FN,linklist,parent_num, child_num);
        end

        function  msg = factor_fun(dist_FN, in_msg, to_node, from_node)
           if dist_FN.type == 0
               msg{1}(1) = dist_FN.sensor_pos_x-in_msg{1}(1);
               msg{1}(2) = in_msg{1}(2);
           else
               msg{1}(1) = dist_FN.sensor_pos_y-in_msg{1}(1);
               msg{1}(2) = in_msg{1}(2);
           end
        
           dist_FN.to_node = [dist_FN.to_node; to_node];
           dist_FN.outbound_msg = [dist_FN.outbound_msg; msg];
           dist_FN.from_node = [dist_FN.from_node; from_node];
           dist_FN.inbound_msg = [dist_FN.inbound_msg; in_msg];
        end

    end
end

