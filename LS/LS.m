clc
clear all
close all

%% Parameters
N = 3;
locations = 3;           % Number of tested locations
sample = 100;               % Number of samples

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
noise_var = noise_std.^2;     % Measurement error in degree

% Targets
rng(1)
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

% LS
results = cell(locations);
errors = zeros(locations);
for targ_iter=1:locations
    fprintf("Location: %d\n", targ_iter)
    noise = sqrt(noise_var)*randn(N,sample);

    % Measured DOA
    theta_samples = repmat(theta(targ_iter,:)',[1 sample]);
    theta_samples = (theta_samples+noise);

    % Measured DOA - mean and variance
    mean_theta = mean(theta_samples,2);
    var_theta = var(theta_samples,[],2);

    % A matrix
    A = [ones(N,1) -tand(mean_theta)];
            
    for i=1:N
        sensor = sensors{1,i};
        b(i,:) = sensor(2)-(sensor(1)*tand(mean_theta(i)));
    end

    % LS
    last_loc = (A'*A)\A'*b;
    last_loc = flip(last_loc);
    last_loc = last_loc';

    % Plot
    for i=1:N
        sensor = sensors{1,i};
        p1 = plot(sensor(1),sensor(2),'r^','MarkerSize', 11);
        txt = ['Sensor ',num2str(i)];
        text(sensor(1),sensor(2),txt);
        hold on
    end
    xlabel('X (meter)');
    ylabel('Y (meter)');
    
    p2 = plot(targets(targ_iter,1),targets(targ_iter,2),'bo','MarkerSize',11)
    text(targets(targ_iter,1),targets(targ_iter,2),'Target')
    p3 = plot(last_loc(1),last_loc(2),'rx','MarkerSize',11)
    legend([p1 p2 p3],{'Sensor' 'Target' 'Estimate'})
    drawnow
    pause
    hold off
    
    % Collect results and errors
    errors(targ_iter,1) = norm(last_loc-targets(targ_iter,:));
    results{targ_iter}=[last_loc(1);last_loc(2)];
end


