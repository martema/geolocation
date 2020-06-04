clc
clear all
close all

%% Parameters
N = 3;
iter = 10;                  % Number of iterations
locations = 1;           % Number of tested locations
sample = 100;               % Number of samples

% Initial position
mean0 = [600 -500];
var0 = [1 1];

% Initial message
in_msg0_x = {[mean0(1) var0(1)]};
in_msg0_y = {[mean0(2) var0(2)]};

in_msg0_x = repmat(in_msg0_x,[1 N]);
in_msg0_y = repmat(in_msg0_y,[1 N]);

% Detection area
x_axis1 = 100;            
x_axis2 = 1100;
y_axis1 = -1000;
y_axis2 = 0;
x_axis = x_axis1:x_axis2;           % Detection area: x axis (meters)
y_axis = y_axis1:y_axis2;           % Detection area: y axis (meters)

% Sensors
sensors = {[100 0], [1100 0], [600 -1000]};     % 3 sensors
%sensors = {[100 0], [1100 0], [600 -1000] [600 -500]}; % 4 sensors 
%sensors = {[100 0], [1100 0], [600 -500], [100 -1000], [1100 -1000]};     % 5 sensors

% Noise
noise_mean = 0;
noise_std = [5];
noise_var = noise_std.^2;     % Measurement error in degree

% Targets
%rng(1)
targets = zeros(locations,2);
targets(:,1) = randi([x_axis1 x_axis2],1,locations);
targets(:,2) = randi([y_axis1 y_axis2],1,locations);

% True DOA
theta = zeros(locations,N);

for i=1:N
    sensor = sensors{1,i};
    delta_x = -(sensor(1)-targets(:,1));
    delta_y = -(sensor(2)-targets(:,2));
    theta(:,i) = atan2d(delta_y,delta_x);       % True DOA

end

results = cell(locations);
error = zeros(locations);

for targ_iter=1:locations
    fprintf("Location: %d\n", targ_iter)
    noise = sqrt(noise_var)*randn(N,sample);

    % Measured DOA
    theta_samples = repmat(theta(targ_iter,:)',[1 sample]);
    theta_samples = (theta_samples+noise);

    % Measured DOA - mean and variance
    mean_theta = mean(theta_samples,2);
    var_theta = var(theta_samples,[],2);

    %% Factor graph
    % Create nodes
    nodes = cell(4,N);

    for i=1:N
        nodes{1,i} = tangent_factor_node([1,i],mean_theta(i),var_theta(i));
        nodes{2,i} = relative_distance_factor_node([2,i],sensors{1,i}(1),sensors{1,i}(2),'A');
        nodes{3,i} = relative_distance_factor_node([3,i],sensors{1,i}(1),sensors{1,i}(2),'B');
    end

    nodes{4,1} = position_factor_node([4 1]);
    nodes{4,2} = position_factor_node([4 2]);

    % Set up connections

    for i=1:N
        nodes{1,i}.setup_link({nodes{2,i},nodes{3,i}},0,2);
        nodes{2,i}.setup_link({nodes{1,i},nodes{4,1}},1,1);
        nodes{3,i}.setup_link({nodes{1,i},nodes{4,2}},1,1);
    end

    nodes{4,1}.setup_link(nodes(2,:),N,0);
    nodes{4,2}.setup_link(nodes(3,:),N,0);

    %% Detection
    in_msg_x = in_msg0_x;
    in_msg_y = in_msg0_y;

    x_est = cell(1,iter+1);
    y_est = cell(1,iter+1);

    x_est(1,1) = in_msg0_x(1);
    y_est(1,1) = in_msg0_y(1);

    for iteration=1:iter
        out_msg_x = nodes{4,1}.factor_fun(in_msg_x, nodes{4,1}.linklist, nodes{4,1}.linklist);
        out_msg_y = nodes{4,2}.factor_fun(in_msg_y, nodes{4,2}.linklist, nodes{4,2}.linklist);

        out_msg_deltax_tot = cell(1,N);
        out_msg_deltay_tot = cell(1,N);

        for i=1:N
            out_msg_deltax = nodes{2,i}.factor_fun(out_msg_x(1,i), nodes{2,i}.linklist(1:nodes{2,i}.parent_num),nodes{2,i}.linklist(nodes{2,i}.parent_num+1:nodes{2,i}.parent_num+nodes{2,i}.child_num));
            out_msg_deltay = nodes{3,i}.factor_fun(out_msg_y(1,i), nodes{3,i}.linklist(1:nodes{3,i}.parent_num),nodes{3,i}.linklist(nodes{3,i}.parent_num+1:nodes{3,i}.parent_num+nodes{3,i}.child_num));
            out_msg_tang = nodes{1,i}.factor_fun([out_msg_deltax ,out_msg_deltay], nodes{1,i}.linklist,nodes{1,i}.linklist);
            out_msg_deltax = nodes{2,i}.factor_fun(out_msg_tang(1,1), nodes{2,i}.linklist(nodes{2,i}.parent_num+1:nodes{2,i}.parent_num+nodes{2,i}.child_num),nodes{2,i}.linklist(1:nodes{2,i}.parent_num));
            out_msg_deltay = nodes{3,i}.factor_fun(out_msg_tang(1,2), nodes{3,i}.linklist(nodes{3,i}.parent_num+1:nodes{3,i}.parent_num+nodes{3,i}.child_num),nodes{3,i}.linklist(1:nodes{3,i}.parent_num));
            out_msg_deltax_tot(1,i) = out_msg_deltax;
            out_msg_deltay_tot(1,i) = out_msg_deltay;
        end
        in_msg_x = out_msg_deltax_tot;
        in_msg_y = out_msg_deltay_tot;

        x_est(1,iteration+1) = nodes{4,1}.position_estimation(out_msg_deltax_tot);
        y_est(1,iteration+1) = nodes{4,2}.position_estimation(out_msg_deltay_tot);
    end

    x_est = cell2mat(x_est');
    y_est = cell2mat(y_est');
    
    % Plot
    for i=1:N
        sensor = sensors{1,i};
        p = plot(sensor(1),sensor(2),'r^','MarkerSize', 11);
        txt = ['Sensor ',num2str(i)];
        text(sensor(1),sensor(2),txt);
        hold on
    end
    xlabel('X (meter)');
    ylabel('Y (meter)');
    
    h = animatedline('Color','g','Marker','x','MarkerSize',11,'LineWidth',2);
    text(targets(targ_iter,1),targets(targ_iter,2),'Target')
    plot(mean0(1),mean0(2),'gx','MarkerSize',11)
    
    for k=1:length(x_est(:,1))
        addpoints(h,x_est(k,1),y_est(k,1));
        drawnow
        pause
    end
   
    % Collect results and errors
    last_loc = [x_est(end,1),y_est(end,1)];
    error(targ_iter,1) = norm(last_loc-targets(targ_iter,:));
    results{targ_iter}=[x_est;y_est];
end

