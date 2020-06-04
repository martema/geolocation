classdef factor_node < handle
% FACTOR_NODE class implements the behaviour of factor nodes in a factor
% graph.  It is derived from handle class so that a factor node object can 
% be referenced by another object.
% 
% Descritpion of methods:
%     FACTOR_NODE instantiate an factor_node object.
%     RESET reset the factor_node object but keeps the linklist.
%     SETUP_LINK initialize the linklist and link_id.
%     FACTOR_FUN, an abstract function, must be defined in derived classes.
%
% Usage:
% Initialize a factor nodes m,n,o with id = 1,2,3 respectively
%     m = factor_node(1);
%     n = factor_node(2);
%     o = factor_node(3);
% Set up the linklist such that m,n,o are linked
%     m.setup_link({n});
%     n.setup_link({m o});
%     o.setup_link({n});
% Reset nodes
%     m.reset();
%     n.reset();
%     o.reset();
%
% Properties:
% id -> Node identification number
% parent_num - Number of parent node
% child_num - Number of child node
% linklist - 1xn cell array - list of parent and child nodes, the first
% part of the cell array are the parent node, the second part are the child
% node
% link_id - 1xn cell array - list of parent and child nodes id, the first
% part of the cell array are the parent node, the second part are the child
% node
% inbound_msg - mxn cell array - i-th row of the cell array contains the
% list of all the inbound messages at i-th iteration
% from_node - mxn cell array - i-th row of the cell array contains the list
% of nodes from which inbound mesages are received at i-th iteration
% from_id - mxn cell array - i-th row of the cell array contains the list
% of nodes id from which inbound mesages are received at i-th iteration
% outbound_msg - mxn cell array - i-th row of the cell array contains the
% list of all the outbound messages at i-th iteration
% to_node - mxn cell array - i-th row of the cell array contains the list
% of nodes to which outbound mesages are sent at i-th iteration
% to_id - mxn cell array - i-th row of the cell array contains the list
% of nodes id to which inbound mesages are sent at i-th iteration

   properties
       id               % id of the object
       parent_num       % number of parent nodes
       child_num        % number of child nodes
       linklist         % linklist, 1xn cell array
       link_id          % link_id, 1xn array
       inbound_msg      % inbound message, nx1 cell array
       from_node        % from node, 1xn cell array
       from_id          % from id, 1xn array
       outbound_msg     % outbound message nx1 cell array
       to_node          % to node, nx1 cell array
       to_id            % to id, 1xn array
   end

   methods
       function FN = factor_node(id)
           % constructor
           FN.id = id;
           FN.linklist = {};
           FN.parent_num = 0;
           FN.child_num  = 0;
           FN.inbound_msg = {};
           FN.outbound_msg = {};
           FN.from_node = {};
           FN.to_node = {};
           FN.from_id = [];
           FN.to_id = [];
           FN.link_id = [];
       end
       
       function reset(FN)
           % reset properties
           FN.inbound_msg = {};
           FN.outbound_msg = {};
           FN.from_node = {};
           FN.to_node = {};
           FN.from_id = [];
           FN.to_id = [];
       end
       
       function setup_link(FN, linklist, parent_num, child_num)
           % update linklist
           if (iscell(linklist) ~= 1) || (size(linklist,1) ~= 1)
               sprintf('Node %0.0f ERROR from setup_link: linklist_parent must be 1xn cell array !!!\n',FN.id)
           else
               FN.linklist = linklist;
               FN.parent_num = parent_num;
               FN.child_num = child_num;
               for i = 1:size(linklist,2)
                   FN.link_id = [FN.link_id linklist{i}.id];
               end
           end
       end
       
       function error_check(FN)
           if isempty(FN.id)
               sprintf('Node %0.0f ERROR from error_check: no node id !!!\n',FN.id)
           elseif isempty(FN.linklist)
               sprintf('Node %0.0f ERROR from error_check: empty linklist !!!\n',FN.id)
           end
       end
       
   end % methods
   
   methods (Abstract = true)
       out_msg = factor_fun(FN, in_msg)
   end

end 
