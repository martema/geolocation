classdef tangent_factor_node < factor_node
    %TANGENT_FACTOR_NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mean_theta
        var_theta 
    end
    
    methods
        function tang_FN = tangent_factor_node(id,mean_theta,var_theta)
            %TANGENT_FACTOR_NODE Construct an instance of this class
            %   Detailed explanation goes here
            tang_FN = tang_FN@factor_node(id);
            tang_FN.mean_theta = mean_theta;
            tang_FN.var_theta = var_theta;
        end

        function reset(tang_FN)
           reset@factor_node(tang_FN);
        end
        
        function setup_link(tang_FN,linklist, parent_num, child_num)
           setup_link@factor_node(tang_FN,linklist,parent_num, child_num);
        end

        function  msg = factor_fun(tang_FN, in_msg, to_node, from_node)
           msg2(1,1) = in_msg{1}(1)*tand(tang_FN.mean_theta);
           msg2(1,2) = in_msg{1}(2)*tand(tang_FN.mean_theta)^2+...
               in_msg{1}(1)^2*tang_FN.var_theta*secd(tang_FN.mean_theta)^4+...
               in_msg{1}(2)*tang_FN.var_theta*secd(tang_FN.mean_theta)^4;
           msg1(1,1) = in_msg{2}(1)*cotd(tang_FN.mean_theta);
           msg1(1,2) = in_msg{2}(2)*cotd(tang_FN.mean_theta)^2+...
               in_msg{2}(1)^2*tang_FN.var_theta*cscd(tang_FN.mean_theta)^4+...
               in_msg{2}(2)*tang_FN.var_theta*cscd(tang_FN.mean_theta)^4;
           
           msg = {msg1 msg2};
           
           tang_FN.to_node = [tang_FN.to_node; to_node];
           tang_FN.outbound_msg = [tang_FN.outbound_msg; msg];
           tang_FN.from_node = [tang_FN.from_node; from_node];
           tang_FN.inbound_msg = [tang_FN.inbound_msg; in_msg];
        end
        
    
    end
end

