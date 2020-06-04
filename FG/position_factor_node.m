classdef position_factor_node < factor_node
    %POSITION_FACTOR_NODE class implements the Estimated Target Position
    %Variable Node (x,y) in the TS-DOA-FG 
    %
    % out_msg = factor_fun(pos_FN, in_msg, to_node, from_node) calculates
    % the messages from the Estimated Target Position Variable Node (x,y),
    % to the Relative Distance Factor Node A_theta and B_theta
    % pos_FN -> actual position factor node instance
    % in_msg - 1xn cell array - Each cell contains a vector
    % [m,sigma], with the messages from the related Relative Distance
    % Factor Nodes (A for x and B for y). The cell array contains a cell
    % for each sensor in the network.
    % to_node - 1xn cell array - List of nodes to which out_msg are sent
    % from_node - 1xn cell array - List of nodes from which in_msg are received
    % out_msg - 1xn cell array - List of outbound messages, each cell
    % contain the messages to the related Relative Distance Factor Node (A
    % for x and B for y). The cell array contains a cell for each sensor in
    % the network
    % 
    % estim_pos = position_estimation(~, in_msg) calculates the estimated
    % position
    % in_msg - 1xn cell array - List of inbound messages from all the
    % sensors
    % estim_pos - 1x1 cell array - Estimated mean and variance on the x or
    % the y axis


    properties
    end
    
    methods
        function pos_FN = position_factor_node(id)
            pos_FN = pos_FN@factor_node(id);
        end

        function reset(pos_FN)
           reset@factor_node(pos_FN);
        end
        
        function setup_link(pos_FN,linklist, parent_num, child_num)
            setup_link@factor_node(pos_FN,linklist,parent_num, child_num);
        end

       %% Calculate outbound messages
        function  out_msg = factor_fun(pos_FN, in_msg, to_node, from_node)
            % First iteration: the outbound message is equal to the inbound
            % message 
            if isempty(pos_FN.to_node)
                out_msg = in_msg;
            % Following iterations
            else
%                 in_msg_mat = cell2mat(in_msg');
%                 for i=1:size(to_node,2)
%                     % i-th sensor variance and mean calculation
%                     temp_in_msg_mat = [in_msg_mat(1:i-1,:); in_msg_mat(i+1:end,:)]; % inbound messages except i-th one      
%                     
%                     % Variance calculation
%                     temp1 = 1./temp_in_msg_mat(:,2);
%                     out_msg{1,i}(2) = 1./(sum(temp1)); 
% 
%                     % Mean calculation
%                     temp2 = temp_in_msg_mat(:,1)./temp_in_msg_mat(:,2);
%                     out_msg{1,i}(1) = out_msg{1,i}(2)*sum(temp2);
%                 end

                  in_msg_mat = cell2mat(in_msg');
                  
                  % Variacne calculation
                  temp1 = 1./in_msg_mat(:,2);
                  temp1_sum = sum(temp1);
                  
                  for i=1:size(to_node,2)
                    out_msg{1,i}(2)=1./(temp1_sum-(1/in_msg_mat(i,2)));
                  end
                  
                  % Mean calculation
                  temp2 = in_msg_mat(:,1)./in_msg_mat(:,2);
                  temp2_sum  = sum(temp2);
                  
                  for i=1:size(to_node,2)
                    out_msg{1,i}(1) = out_msg{1,i}(2)*(temp2_sum-(in_msg_mat(i,1)./in_msg_mat(i,2)));
                  end
%                 for i=1:size(to_node,2)
%                     % i-th sensor variance and mean calculation
%                     temp_in_msg_mat = [in_msg_mat(1:i-1,:); in_msg_mat(i+1:end,:)]; % inbound messages except i-th one      
%                     
%                     % Variance calculation
%                     temp1 = 1./temp_in_msg_mat(:,2);
%                     out_msg{1,i}(2) = 1./(sum(temp1)); 
% 
%                     % Mean calculation
%                     temp2 = temp_in_msg_mat(:,1)./temp_in_msg_mat(:,2);
%                     out_msg{1,i}(1) = out_msg{1,i}(2)*sum(temp2);
%                 end
            end
            
            % Saving messages and nodes 
            pos_FN.to_node = [pos_FN.to_node; to_node];
            pos_FN.outbound_msg = [pos_FN.outbound_msg; out_msg];
            pos_FN.from_node = [pos_FN.from_node; from_node];
            pos_FN.inbound_msg = [pos_FN.inbound_msg; in_msg];
        end

        %% Calculate estimated position
         function  estim_pos = position_estimation(~, in_msg)
             in_msg_mat = cell2mat(in_msg');
             
             % Variance calculation
             temp1 = 1./in_msg_mat(:,2);
             estim_pos{1}(2) = 1./(sum(temp1));

             % Mean calculation
             temp2 = in_msg_mat(:,1)./in_msg_mat(:,2);
             estim_pos{1}(1) = estim_pos{1}(2)*sum(temp2);

        end
    end
end


