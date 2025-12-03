% Animation for MRCLAM Dataset (using struct 'data' created by loadMRCLAMdataSet.m)

% Options %
start_timestep = 1;
end_timestep   = timesteps;
timesteps_per_frame = 50;
pause_time_between_frames = 0.01;
draw_measurements = 0;
% END OPTIONS %

n_robots = 5;
n_landmarks = size(data.Landmark_Groundtruth,1);

% Colors
colour = [
    1 0 0;
    0 0.75 0;
    0 0 1;
    1 0.5 0.25;
    1 0.5 1
];

for i = 6:6+n_landmarks
    colour(i,:) = [0.3 0.3 0.3];
end

figure('Name','Dataset Groundtruth','Renderer','OpenGL');
set(gcf,'Position',[1300 1 630 950]);

plotHandles_robot_gt = zeros(n_robots,1);
plotHandles_landmark_gt = zeros(n_landmarks,1);

r_robot = 0.165;
d_robot = 2 * r_robot;
r_landmark = 0.055;
d_landmark = 2 * r_landmark;

% Build robot state matrix
Robot = [];
for i = 1:n_robots
    Robot = [Robot data.Robot(i).Groundtruth(:,2:4)];
end

% INITIAL DRAWING
n_measurements = zeros(n_robots,1);
for i = 1:n_robots
    x  = Robot(1,3*(i-1)+1);
    y  = Robot(1,3*(i-1)+2);
    th = Robot(1,3*(i-1)+3);

    x1 = x + d_robot * cos(th);
    y1 = y + d_robot * sin(th);

    plotHandles_robot_gt(i) = rectangle(...
        'Position',[x-r_robot, y-r_robot, d_robot, d_robot],...
        'Curvature',[1,1],...
        'FaceColor',colour(i,:),...
        'LineWidth',1);

    line([x x1],[y y1],'Color','k');

    n_measurements(i) = size(data.Robot(i).Measurement,1);
end

% LANDMARKS
for i = 1:n_landmarks
    x = data.Landmark_Groundtruth(i,2);
    y = data.Landmark_Groundtruth(i,3);

    plotHandles_landmark_gt(i) = rectangle(...
        'Position',[x-r_landmark, y-r_landmark, d_landmark, d_landmark],...
        'Curvature',[1,1],...
        'FaceColor',colour(i+5,:),...
        'LineWidth',1);
end

axis square;
axis equal;
axis([-2 6 -6 7]);
set(gca,'XTick',(-10:2:10)');

% MEASUREMENT INDICES
measurement_time_index = ones(n_robots,1);

for i = 1:n_robots
    M = data.Robot(i).Measurement;
    idx = find(M(:,1) >= start_timestep*sample_time,1,'first');
    if isempty(idx)
        measurement_time_index(i) = n_measurements(i) + 1;
    else
        measurement_time_index(i) = idx;
    end
end

% MAIN LOOP
for k = start_timestep:end_timestep
    t = k * sample_time;

    if mod(k,timesteps_per_frame) == 0
        delete(findobj('Type','line'));
    end

    % Update each robot
    for i = 1:n_robots
        x  = Robot(k,3*(i-1)+1);
        y  = Robot(k,3*(i-1)+2);
        th = Robot(k,3*(i-1)+3);

        if mod(k,timesteps_per_frame) == 0
            x1 = x + d_robot * cos(th);
            y1 = y + d_robot * sin(th);

            set(plotHandles_robot_gt(i),...
                'Position',[x-r_robot y-r_robot d_robot d_robot]);
            line([x x1],[y y1],'Color','k');
        end

        % Measurements
        if draw_measurements
            M = data.Robot(i).Measurement;

            while measurement_time_index(i) <= n_measurements(i) && ...
                  M(measurement_time_index(i),1) <= t

                measure_id = M(measurement_time_index(i),2);
                measure_r  = M(measurement_time_index(i),3);
                measure_b  = M(measurement_time_index(i),4);

                x1 = x + measure_r * cos(measure_b + th);
                y1 = y + measure_r * sin(measure_b + th);

                line([x x1],[y y1],'Color',colour(i,:),'LineWidth',1);

                measurement_time_index(i) = measurement_time_index(i) + 1;
            end
        end
    end

    % Draw timestamp
    if mod(k,timesteps_per_frame)==0
        delete(findobj('Type','text'));
        text(1.5,6.5, sprintf("k=%5d   t=%5.2f[s]",k,t));
        pause(pause_time_between_frames);
    else
        if draw_measurements
            pause(0.001);
        end
    end
end
