function fastslam_known_correspondences()
%FASTSLAM_KNOWN_CORRESPONDENCES
    loadMRCLAMdataSet;

    robot_id = 1;

    odom = data.Robot(robot_id).Odometry;        % [t v w]
    meas = data.Robot(robot_id).Measurement;     % [t barcode range bearing]

    % Tag data: 1 = odom, 2 = measurement
    odom_full = [odom(:,1:3), zeros(size(odom,1),1), ones(size(odom,1),1)]; % [t v w 0 type]
    meas_full = [meas, 2*ones(size(meas,1),1)];                             % [t bc r b type]
    timeline  = sortrows([odom_full; meas_full], 1);
    T = size(timeline,1);


    % Setup (matches your init_estimator choices)
    num_landmarks = 15;

    % Barcodes list (subject -> barcode)
    barcode_list = data.Barcodes(:,2);
    
    fs_data = struct();
    fs_data.barcodes = barcode_list(:);
    fs_data.num_landmarks = num_landmarks;

    robot_gt_all = data.Robot(robot_id).Groundtruth;          % [t x y th]
    landmarks_gt = data.Landmark_Groundtruth(:,2:3);          % [x y]

    params = struct();
    params.N = 50;
    params.R = diag([0.15^2, (5*pi/180)^2]);      
    params.Q = diag([0.02^2, 0.02^2, (2*pi/180)^2]);

    params.resample_thresh = 0.5;
    params.initPose = robot_gt_all(1,2:4)';                   
    params.k_w = 1.05;                                        

    est = fastslam_init(fs_data, params);

    maxBC = max(est.barcodes);
    bc2lm = zeros(maxBC,1,'int16');
    for subj = 6:numel(est.barcodes)
        bc = est.barcodes(subj);
        if bc > 0 && bc <= maxBC
            bc2lm(bc) = int16(subj - 5); % landmarks 1..15
        end
    end
    est.bc2lm = bc2lm;
    est.maxBC = maxBC;

    %Control state
    t_prev = timeline(1,1);
    v = 0;
    w = 0;

    %Error accumulators
    robot_err_sq_sum = zeros(3,1);
    robot_err_count  = 0;

    landmark_err_sq_sum = zeros(num_landmarks,1);
    landmark_err_count  = zeros(num_landmarks,1);

     % Evaluation stuff %

    robot_history = zeros(size(timeline,1), 3);
    robot_history(1,:) = params.initPose';

    robot_gt_history_aligned= zeros(size(timeline,1), 3);
    robot_gt_history_aligned(1,:) = params.initPose';
    h=1;

    %Main loop
    k = 2;

    while k <= T
        t = timeline(k,1);
        dt = t - t_prev;

        % PREDICT
        if dt > 1e-6
            w_used =  params.k_w*w;           % APPLY yaw fix HERE
            est = fastslam_predict(est, v, w_used, dt);
        end

        if timeline(k,5) == 2
            kk = k;

            % Count valid landmark measurements at this timestamp
            nValid = 0;
            while kk <= T && timeline(kk,1) == t && timeline(kk,5) == 2
                bc = timeline(kk,2);

                % EKF-style barcode recognition:
                lm_index = find(barcode_list == bc, 1, 'first');
                if ~isempty(lm_index) && lm_index >= 6
                    nValid = nValid + 1;
                end

                kk = kk + 1;
            end

            if nValid > 0
                z_batch = zeros(nValid,3);
                kk2 = k;
                j = 1;
                while kk2 <= T && timeline(kk2,1) == t && timeline(kk2,5) == 2
                    bc = timeline(kk2,2);

                    lm_index = find(barcode_list == bc, 1, 'first');
                    if ~isempty(lm_index) && lm_index >= 6
                        z_batch(j,1) = bc;
                        z_batch(j,2) = timeline(kk2,3);  % range
                        z_batch(j,3) = timeline(kk2,4);  % bearing
                        j = j + 1;
                    end

                    kk2 = kk2 + 1;
                end

                if ~isempty(z_batch)
                    est = fastslam_update(est, z_batch);
                end
            end

            k = kk;

        else
            v = timeline(k,2);
            w = timeline(k,3);
            k = k + 1;
        end

        t_prev = t;

        % Error computation
        gt = interp_gt(robot_gt_all, t);

        % Pose estimate
        xhat = kmeans_mode_pose(est, 2);

        ao = get_avg_particle_pose(est);

        % adding the lines to save the history of the poses 
        h=h+1;
        robot_history(h, :)=ao';
        robot_gt_history_aligned(h, :)=gt;
        
        e = xhat - gt;
        e(3) = wrapToPi(e(3));
        robot_err_sq_sum = robot_err_sq_sum + e.^2;
        robot_err_count  = robot_err_count + 1;

        [~, idxMAP] = max([est.particles.w]);
        p = est.particles(idxMAP);

        seenIdx = find(p.seen);
        if ~isempty(seenIdx)
            dx = p.lm(1,seenIdx)' - landmarks_gt(seenIdx,1);
            dy = p.lm(2,seenIdx)' - landmarks_gt(seenIdx,2);
            landmark_err_sq_sum(seenIdx) = landmark_err_sq_sum(seenIdx) + (dx.^2 + dy.^2);
            landmark_err_count(seenIdx)  = landmark_err_count(seenIdx) + 1;
        end
    end

%% RMSE final map
final_landmark_err = zeros(num_landmarks,1);
final_landmark_count = 0;
for j = 1:params.N
    p = est.particles(j);
    for i = 1:num_landmarks
            dx = p.lm(1,i) - landmarks_gt(i,1);
            dy = p.lm(2,i) - landmarks_gt(i,2);
            final_landmark_err(i) = final_landmark_err(i) + sqrt(dx^2 + dy^2);
            final_landmark_count = final_landmark_count + 1;
    end
end
final_landmark_rmse = mean(final_landmark_err)/params.N;

fprintf('\nRMSE landmarks (final map only) = %.3f m\n', final_landmark_rmse);

plot_landmark_map_fastslam(p.lm, landmarks_gt, num_landmarks);

    %% RPE
    delta=100;
    [rmse_rpe, trans_error]=RPE(robot_history,robot_gt_history_aligned,h,delta);
    plot_RPE(trans_error,delta);
    fprintf('\n--- RPE ---\n');
    fprintf('RPE RMSE %.3f\n', rmse_rpe);
    
    %% ATE
    disp('--- ATE ---');
    [ate_rmse, ate_errors, aligned_traj_xy] = ATE(robot_gt_history_aligned, robot_history, h);
    fprintf('ATE RMSE: %.4f m\n', ate_rmse);
    plot_ATE(robot_gt_history_aligned, aligned_traj_xy, ate_errors, ate_rmse);
end

% helpers

function gt = interp_gt(gt_all, t)
% Interpolate GT [t x y th] to timestamp t; angle via sin/cos
    tg = gt_all(:,1);
    if t <= tg(1)
        gt = gt_all(1,2:4)'; return;
    elseif t >= tg(end)
        gt = gt_all(end,2:4)'; return;
    end

    x  = interp1(tg, gt_all(:,2), t, 'linear');
    y  = interp1(tg, gt_all(:,3), t, 'linear');

    sth = interp1(tg, sin(gt_all(:,4)), t, 'linear');
    cth = interp1(tg, cos(gt_all(:,4)), t, 'linear');
    th  = atan2(sth, cth);

    gt = [x; y; th];
end

function xhat = kmeans_mode_pose(est, K)
% Pick dominant pose mode using K-means on (x,y), then weighted circular mean for theta.
% This avoids "averaging between symmetric hypotheses".

    N = numel(est.particles);
    X = zeros(N,3);
    w = zeros(N,1);
    for i = 1:N
        X(i,:) = est.particles(i).x(:).';
        w(i)   = est.particles(i).w;
    end

    sw = sum(w);
    if sw <= 0 || ~isfinite(sw)
        w = ones(N,1)/N;
    else
        w = w / sw;
    end

    % Subsample for speed
    M = min(1500, N);
    idx = randsample(N, M, true, w);
    XY  = X(idx,1:2);

    % If kmeans fails, fall back to MAP
    try
        lab = kmeans(XY, K, 'Replicates', 3, 'MaxIter', 100);
    catch
        [~,iMAP] = max(w);
        xhat = X(iMAP,:)';
        xhat(3) = wrapToPi(xhat(3));
        return;
    end

    clusterW = zeros(K,1);
    for j = 1:K
        clusterW(j) = sum(w(idx(lab==j)));
    end
    [~,jBest] = max(clusterW);

    sel = idx(lab==jBest);
    ww  = w(sel);
    ww  = ww / sum(ww);

    mx = sum(ww .* X(sel,1));
    my = sum(ww .* X(sel,2));

    c  = sum(ww .* cos(X(sel,3)));
    s  = sum(ww .* sin(X(sel,3)));
    mth = atan2(s,c);

    xhat = [mx; my; wrapToPi(mth)];
end

function ao = get_avg_particle_pose(est)

    N = numel(est.particles);

    X  = zeros(N,1);
    Y  = zeros(N,1);
    TH = zeros(N,1);

    for i = 1:N
        X(i)  = est.particles(i).x(1);
        Y(i)  = est.particles(i).x(2);
        TH(i) = est.particles(i).x(3);
    end

    mx = mean(X);
    my = mean(Y);

    c = mean(cos(TH));
    s = mean(sin(TH));
    mth = atan2(s, c);

    ao = [mx; my; wrapToPi(mth)];
end