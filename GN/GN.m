clear all
close all

%% Parameters
N = 3;
iter = 5;                  % Number of iterations
trial = 1;                % Number of trial
locations = 1;           % Number of tested locations
sample = 100;               % Number of samples

% Initial position
mean0 = [0 0];
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
noise_std = [10];
noise_var_deg = noise_std.^2;     % Measurement error in degree
noise_var = noise_var_deg/360*2*pi;

% Targets
rng(1)
targets = zeros(locations,2);
targets(:,1) = randi([x_axis1 x_axis2],1,locations);
targets(:,2) = randi([y_axis1 y_axis2],1,locations);

% True DOA
theta_deg = zeros(locations,N);

for i=1:N
    sensor = sensors{1,i};
    delta_x = -(sensor(1)-targets(:,1));
    delta_y = -(sensor(2)-targets(:,2));
    theta_deg(:,i) = atan2d(delta_y,delta_x);       % True DOA
    theta(:,i) = theta_deg(:,i)/360*2*pi;
end

% GN
results = cell(locations);
errors = zeros(locations);
for targ_iter=1:locations
    fprintf("Location: %d\n", targ_iter)
        noise_deg = sqrt(noise_var_deg)*randn(N,sample);

        % Measured DOA
        theta_samples_deg = repmat(theta_deg(targ_iter,:)',[1 sample]);
        theta_samples_deg = (theta_samples_deg+noise_deg);

        % Measured DOA - mean and variance
        mean_theta_deg = mean(theta_samples_deg,2);
        var_theta_deg = var(theta_samples_deg,[],2);

        % LS initial position
        A = [ones(N,1) -tand(mean_theta_deg)];

        for i=1:N
            sensor = sensors{1,i};
            b(i,:) = sensor(2)-(sensor(1)*tand(mean_theta_deg(i)));
        end

        init_loc = (A'*A)\A'*b;
        init_loc = flip(init_loc);
        init_loc = init_loc';

        % Initial guess
        x0 = init_loc(1);
        y0 = init_loc(2);

        theta0 = zeros(N,1);

        for i=1:N
            sensor = sensors{1,i};
            delta_x = -(sensor(1)-x0);
            delta_y = -(sensor(2)-y0);
            theta0(i,1) = atan2d(delta_y,delta_x);       % Initial guess
            theta0(i,1) = theta0(i,1)/360*2*pi;
        end
            
        % GN

        % Measured DOA
        theta_samples = theta_samples_deg/360*2*pi;

        % Measured DOA - mean and variance
        mean_theta = mean_theta_deg/360*2*pi;
        var_theta = var_theta_deg/360*2*pi;
        covar = cov(theta_samples');
        theta_q = theta0; % theta at the q-iteration

        loc = zeros(2,iter+1);
        loc(1,1) = x0;
        loc(2,1) = y0;
            
        for iteration=1:iter
            % Jacobian
            sensors_mat = cell2mat(sensors');
            J_temp = (sensors_mat-loc(:,iteration)');
            distance = sum(J_temp.^2,2);
            J_temp2 = [J_temp(:,2) -(J_temp(:,1))];
            distance = [distance distance];
            J = J_temp2./distance;

            % Position at q+1
            loc(:,iteration+1) = loc(:,iteration)+pinv(J'*pinv(covar)*J)*J'*pinv(covar)*(mean_theta-theta_q);
            
            for i=1:N
                sensor = sensors{1,i};
                delta_x = -(sensor(1)-loc(1,iteration+1));
                delta_y = -(sensor(2)-loc(2,iteration+1));
                theta_q(i,1) = atan2d(delta_y,delta_x);
                theta_q(i,1) = theta_q(i,1)/360*2*pi;% Initial guess
            end
            
            
        end 
              
                
        % Convergence check
        last_loc = [loc(1,end),loc(2,end)];

        if (last_loc(1)<-300) || (last_loc(1)>1500) || (last_loc(2)<-1400) || (last_loc(2)>400)
            last_loc = [loc(1,1),loc(2,1)];
        end
        
        loc = loc';
        
        % Plots
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
    
        for k=1:length(loc(:,1))
            addpoints(h,loc(k,1),loc(k,2));
            drawnow
            pause
        end
            
        % Collect results and errors
        results{targ_iter}=[last_loc(1);last_loc(2)];
        errors(targ_iter,1) = norm(last_loc-targets(targ_iter,:));
end


