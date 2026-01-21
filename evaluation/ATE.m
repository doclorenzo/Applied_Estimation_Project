function [ate_rmse, ate_errors, aligned_est_xy] = ATE(gt_pose, est_pose, idx)

    if size(gt_pose, 2) == 3
        gt_xy = gt_pose(1:idx, 1:2)';   
        est_xy = est_pose(1:idx, 1:2)'; 
    else
        gt_xy = gt_pose(1:2, 1:idx);
        est_xy = est_pose(1:2, 1:idx);
    end

    % HORN
    % centroids 
    mu_gt = mean(gt_xy, 2);
    mu_est = mean(est_xy, 2);
    
    gt_centered = gt_xy - mu_gt;
    est_centered = est_xy - mu_est;
    
    % cov matrix W
    W = est_centered * gt_centered';
    
    % SVD
    [U, ~, V] = svd(W);
    
    R = V * U';
    
    if det(R) < 0
        V(:, 2) = -V(:, 2);
        R = V * U';
    end
    
    t = mu_gt - R * mu_est;    
    aligned_est_xy = R * est_xy + t;
    
    % ERRORS
    diff = gt_xy - aligned_est_xy;
    
    ate_errors = sqrt(sum(diff.^2, 1));
    
    % RMSE
    ate_rmse = sqrt(mean(ate_errors.^2));
end