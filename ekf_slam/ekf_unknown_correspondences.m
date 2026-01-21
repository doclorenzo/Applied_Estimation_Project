function [mu, Sigma] = ekf_unknown_correspondences()

loadMRCLAMdataSet;

robot_id = 2;

odom = data.Robot(robot_id).Odometry;      
meas = data.Robot(robot_id).Measurement;   

% Tag data: 1 = odom, 2 = measurement
odom_full = [odom(:,1:3), zeros(size(odom,1),1), ones(size(odom,1),1)];
meas_full = [meas, 2*ones(size(meas,1),1)];

timeline = sortrows([odom_full; meas_full], 1);

% setup
num_landmarks = 0;
mu = zeros(3, 1);
Sigma = eye(3) * 0.01;

% Meas noise
R = diag([0.01, 0.05]);

% Groundtruth 
robot_gt_all = data.Robot(robot_id).Groundtruth;
landmarks_gt = data.Landmark_Groundtruth(:,2:3);

mu(1:3) = robot_gt_all(1,2:4)';

t_prev = timeline(1,1);
v = 0;
w = 0;

barcode_list = data.Barcodes(:,2);

%for RMSE)
landmark_true_ids = [];

% Error 
robot_err_sq_sum = zeros(3,1);
robot_err_count = 0;

landmark_err_sq_sum = [];
landmark_err_count = [];

MAHAL_THRESHOLD = 19.21;

% MAIN
for k = 2:size(timeline,1)
    t = timeline(k,1);
    dt = t - t_prev;

    %  PREDICT 
    if dt > 0.001
        [mu, Sigma] = ekf_predict(mu, Sigma, v, w, dt);
    elseif timeline(k,5) == 1
        continue;
    end

    %  UPDATE 
    if timeline(k,5) == 2  
        barcode_num = timeline(k,2);
        true_lm_index = find(barcode_list == barcode_num);

        if ~isempty(true_lm_index) && true_lm_index >= 6
            r = timeline(k,3);
            b = timeline(k,4);
            true_lm_index = true_lm_index - 5;

            x_r = mu(1);
            y_r = mu(2);
            theta = mu(3);

            %  DATA ASSOCIATION 
            mahal_best = inf;
            innovation_best = [];
            S_best = [];
            H_best = [];
            j_best = -1;


            for j = 1:num_landmarks
                lm_x = mu(3 + 2*j - 1);
                lm_y = mu(3 + 2*j);

                dx = lm_x - x_r;
                dy = lm_y - y_r;
                q = dx^2 + dy^2;

                z_hat = [sqrt(q); atan2(dy, dx) - theta];

                % Jacobian H
                sqrt_q = sqrt(q);
                H = zeros(2, length(mu));
                H(1,1) = -dx / sqrt_q;
                H(1,2) = -dy / sqrt_q;
                H(1,3) = 0;
                H(1, 3+2*j-1) = dx / sqrt_q;
                H(1, 3+2*j)   = dy / sqrt_q;

                H(2,1) = dy / q;
                H(2,2) = -dx / q;
                H(2,3) = -1;
                H(2, 3+2*j-1) = -dy / q;
                H(2, 3+2*j)   = dx / q;

                % Innovation
                z = [r; b];
                y = z - z_hat;
                y(2) = wrapToPi(y(2));

                S = H * Sigma * H' + R;

                % Mahalanobis distance
                mahal = y' * (S \ y);

                if mahal < mahal_best
                    mahal_best = mahal;
                    S_best = S;
                    innovation_best = y;
                    H_best = H;
                    j_best = j;
                end
            end

          
            if mahal_best < MAHAL_THRESHOLD && num_landmarks > 0
                % UPDATE
                K = Sigma * H_best' / S_best;
                mu = mu + K * innovation_best;
                mu(3) = wrapToPi(mu(3));
                Sigma = (eye(length(mu)) - K * H_best) * Sigma;

            else
                % NEW LANDMARK
                lm_x = x_r + r*cos(theta + b);
                lm_y = y_r + r*sin(theta + b);

                mu = [mu; lm_x; lm_y];

                Gx = [1, 0, -r*sin(theta+b);
                      0, 1,  r*cos(theta+b)];
                Gz = [cos(theta+b), -r*sin(theta+b);
                      sin(theta+b),  r*cos(theta+b)];

                n = length(mu);
                Sigma_new = zeros(n, n);
                Sigma_new(1:n-2, 1:n-2) = Sigma;

                Sigma_new(n-1:n, n-1:n) = Gx * Sigma(1:3,1:3) * Gx' + Gz * R * Gz';

                Sigma_new(1:3, n-1:n) = Sigma(1:3, 1:3) * Gx';
                Sigma_new(n-1:n, 1:3) = Gx * Sigma(1:3, 1:3);

                if num_landmarks > 0
                    Sigma_new(4:n-2, n-1:n) = Sigma(4:n-2, 1:3) * Gx';
                    Sigma_new(n-1:n, 4:n-2) = Gx * Sigma(1:3, 4:n-2);
                end

                Sigma = Sigma_new;
                num_landmarks = num_landmarks + 1;

                landmark_true_ids = [landmark_true_ids; true_lm_index];
                landmark_err_sq_sum = [landmark_err_sq_sum; 0];
                landmark_err_count = [landmark_err_count; 0];

                fprintf('[t=%.1f] new landmark %d\n', t, num_landmarks);
            end
        end

    else
        v = timeline(k,2);
        w = timeline(k,3);
    end

    t_prev = t;

    %ERROR 
    [~, idx_gt] = min(abs(robot_gt_all(:,1) - t));
    robot_err = mu(1:3) - robot_gt_all(idx_gt,2:4)';
    robot_err(3) = wrapToPi(robot_err(3));

    robot_err_sq_sum = robot_err_sq_sum + robot_err.^2;
    robot_err_count = robot_err_count + 1;

    for j = 1:num_landmarks
        true_id = landmark_true_ids(j);
        dx = mu(3+2*j-1) - landmarks_gt(true_id,1);
        dy = mu(3+2*j)   - landmarks_gt(true_id,2);
        landmark_err_sq_sum(j) = landmark_err_sq_sum(j) + dx^2 + dy^2;
        landmark_err_count(j) = landmark_err_count(j) + 1;
    end
end

%  RMSE 
robot_rmse = sqrt(robot_err_sq_sum / robot_err_count);

fprintf('\n--- RMSE robot ---\n');
fprintf('x = %.4f m, y = %.4f m, theta = %.4f rad\n', ...
        robot_rmse(1), robot_rmse(2), robot_rmse(3));

if num_landmarks > 0
    landmark_rmse = sqrt(landmark_err_sq_sum ./ landmark_err_count);
    fprintf('\n--- Landmarks ---\n');
    fprintf('total landmark found: %d\n', num_landmarks);
    fprintf('RMSE = %.4f m\n', mean(landmark_rmse));
else
    fprintf('\No landmark found\n');
end

end