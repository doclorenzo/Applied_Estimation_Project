function plot_ATE_results(gt_pose, aligned_est_xy, ate_errors, ate_rmse)
    
    if size(gt_pose, 2) >= 2
        gt_x = gt_pose(:, 1);
        gt_y = gt_pose(:, 2);
    else
        gt_x = gt_pose(1, :)';
        gt_y = gt_pose(2, :)';
    end
    

    est_x = aligned_est_xy(1, :);
    est_y = aligned_est_xy(2, :);
    

    f = figure('Name', 'ATE Analysis Result', 'Color', 'w', 'Position', [100, 100, 1200, 500]);
    
    set(f, 'Renderer', 'painters'); 

    subplot(1, 2, 1);
    
    if ~isempty(gt_x)
        plot(gt_x, gt_y, 'k--', 'LineWidth', 0.5, 'DisplayName', 'Ground Truth'); 
        hold on;
    end
    
    if ~isempty(est_x)
        plot(est_x, est_y, 'b-', 'LineWidth', 0.5, 'DisplayName', 'Estimation');
    end
    
    axis equal; grid on;
    title('Trajectories', 'FontSize', 12);
    xlabel('X [m]'); ylabel('Y [m]');
    legend('Location', 'best');
    
    subplot(1, 2, 2);
    valid_err = ~isnan(ate_errors);
    clean_errors = ate_errors(valid_err);
    
    plot(clean_errors, 'Color', [0.85, 0.325, 0.098], 'LineWidth', 0.5); hold on;
    
    yline(ate_rmse, '--b', ['RMSE: ' num2str(ate_rmse, '%.3f') ' m'], 'LineWidth', 1); 
    
    if ~isempty(clean_errors)
        [max_err, idx_max] = max(clean_errors);
        plot(idx_max, max_err, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
        text(idx_max, max_err, sprintf(' Max: %.2fm', max_err), 'VerticalAlignment', 'bottom');
        xlim([1, length(clean_errors)]);
    end

    title('ATE in time', 'FontSize', 12);
    xlabel('samples'); ylabel('Error [m]');
    grid on;
    
    sgtitle(sprintf('ATE Evaluation - RMSE: %.4f m', ate_rmse), 'FontSize', 14, 'FontWeight', 'bold');
end