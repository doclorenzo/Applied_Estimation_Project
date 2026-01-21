function [mu, Sigma] = ekf_known_correspondences()

loadMRCLAMdataSet;

robot_id = 2;

odom = data.Robot(robot_id).Odometry;       
meas = data.Robot(robot_id).Measurement;    

% Tag data: 1 = odom, 2 = measurement
odom_full = [odom(:,1:3), zeros(size(odom,1),1), ones(size(odom,1),1)];
meas_full = [meas, 2*ones(size(meas,1),1)];

timeline = sortrows([odom_full; meas_full], 1);

% setup
num_landmarks = 15;
mu = zeros(3 + 2*num_landmarks, 1);
Sigma = eye(length(mu)) * 0;
Sigma(4:end,4:end) = 10000 * eye(2*num_landmarks);

%  Groundtruth 
robot_gt_all = data.Robot(robot_id).Groundtruth;   
landmarks_gt = data.Landmark_Groundtruth(:,2:3); 

% Initialize robot pose
mu(1:3) = robot_gt_all(1,2:4)';

t_prev = timeline(1,1);
v = 0;
w = 0;

seen = zeros(num_landmarks,1);
barcode_list = data.Barcodes(:,2);

% Error 
robot_err_sq_sum = zeros(3,1);
robot_err_count = 0;

landmark_err_sq_sum = zeros(num_landmarks,1);
landmark_err_count  = zeros(num_landmarks,1);

robot_history = zeros(size(timeline,1), 3);
robot_history(1,:) = mu(1:3)';
robot_gt_history_aligned= zeros(size(timeline,1), 3);
robot_gt_history_aligned(1,:) = mu(1:3)';
robot_timestamps_history = zeros(size(timeline, 1), 1);

valid_steps_idx=1;

% MAIN 
for k = 2:size(timeline,1)
    t = timeline(k,1);
    dt = t - t_prev;

    %  PREDICT 
    if dt > 0.001
        [mu, Sigma] = ekf_predict(mu, Sigma, v, w, dt);
    elseif timeline(k,5)==1
        continue;
    end

    valid_steps_idx=valid_steps_idx+1;

    %  UPDATE
    if timeline(k,5) == 2   % measurement
        barcode_num = timeline(k,2);
        lm_index = find(barcode_list == barcode_num);

        if ~isempty(lm_index) && lm_index >= 6
            r = timeline(k,3);
            b = timeline(k,4);
            lm_index = lm_index - 5;

            if seen(lm_index) == 0
                
                %init landmark
                x_r = mu(1);
                y_r = mu(2);
                theta = mu(3);

                mu(3 + 2*lm_index - 1) = x_r + r*cos(theta + b);
                mu(3 + 2*lm_index) = y_r + r*sin(theta + b);

                seen(lm_index) = 1;
                
            end
            [mu, Sigma] = ekf_update(mu, Sigma, r, b, lm_index);
        end
    else
        v = timeline(k,2);
        w = timeline(k,3);
    end

    t_prev = t;

    % Robot error with interpolation
    gt = interp_gt(robot_gt_all, t);

    robot_history(valid_steps_idx, :)=mu(1:3)';
    robot_gt_history_aligned(valid_steps_idx, :) = gt';
    robot_timestamps_history(valid_steps_idx)=t;

    robot_err = mu(1:3) - gt;
    robot_err(3) = wrapToPi(robot_err(3));

    robot_err_sq_sum = robot_err_sq_sum + robot_err.^2;
    robot_err_count  = robot_err_count + 1;

    % Landmark error (only seen ones)
    for i = 1:num_landmarks
        if seen(i)
            dx = mu(3+2*i-1) - landmarks_gt(i,1);
            dy = mu(3+2*i)   - landmarks_gt(i,2);
            landmark_err_sq_sum(i) = landmark_err_sq_sum(i) + dx^2 + dy^2;
            landmark_err_count(i)  = landmark_err_count(i) + 1;
        end
    end
end

%% RMSE 
robot_rmse = sqrt(robot_err_sq_sum / robot_err_count);
fprintf('\n--- robot RMSE ---\n');
fprintf('x = %.3f m, y = %.3f m, theta = %.3f rad\n', ...
        robot_rmse(1), robot_rmse(2), robot_rmse(3));
valid_lm = landmark_err_count > 0;
landmark_rmse = sqrt(landmark_err_sq_sum(valid_lm) ./ landmark_err_count(valid_lm));
fprintf('RMSE landmarks = %.3f m\n', mean(landmark_rmse));

%% RMSE final map
final_landmark_err = zeros(num_landmarks,1);
final_landmark_count = 0;
for i = 1:num_landmarks
    if seen(i)  
        dx = mu(3 + 2*i - 1) - landmarks_gt(i,1);
        dy = mu(3 + 2*i)     - landmarks_gt(i,2);
        final_landmark_err(i) = sqrt(dx^2 + dy^2);
        final_landmark_count = final_landmark_count + 1;
    end
end
final_landmark_rmse = mean(final_landmark_err(seen == 1));
fprintf('\nRMSE landmarks (final map only) = %.3f m\n', final_landmark_rmse);
plot_landmark_map(mu, landmarks_gt, seen, num_landmarks);

%% RPE
delta=100;
[rmse_rpe, trans_error]=RPE(robot_history,robot_gt_history_aligned,valid_steps_idx,delta);
plot_RPE(trans_error,delta);
fprintf('\n--- RPE ---\n');
fprintf('RPE RMSE %.3f\n', rmse_rpe);

%% ATE
disp('--- ATE ---');
[ate_rmse, ate_errors, aligned_traj_xy] = ATE(robot_gt_history_aligned, robot_history, valid_steps_idx);
fprintf('ATE RMSE: %.4f m\n', ate_rmse);
plot_ATE(robot_gt_history_aligned, aligned_traj_xy, ate_errors, ate_rmse);

end

% Helper function for interpolation
function gt = interp_gt(gt_all, t)
% Interpolate GT [t x y th] to timestamp t; angle via sin/cos
    tg = gt_all(:,1);
    if t <= tg(1)
        gt = gt_all(1,2:4)'; 
        return;
    elseif t >= tg(end)
        gt = gt_all(end,2:4)'; 
        return;
    end

    x  = interp1(tg, gt_all(:,2), t, 'linear');
    y  = interp1(tg, gt_all(:,3), t, 'linear');

    sth = interp1(tg, sin(gt_all(:,4)), t, 'linear');
    cth = interp1(tg, cos(gt_all(:,4)), t, 'linear');
    th  = atan2(sth, cth);

    gt = [x; y; th];
end