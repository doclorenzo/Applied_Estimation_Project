function plot_landmark_map_fastslam(lm, landmarks_gt, num_landmarks)

    figure('Name', 'FastSLAM Landmark Map', 'Position', [100, 100, 900, 800]);
    hold on;
    grid on;
    axis equal;
    
    %  groundtruth landmarks
    plot(landmarks_gt(:,1), landmarks_gt(:,2), 'go', ...
         'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', [0.7 1 0.7]);
    
    %  estimated landmarks and error lines
    for i = 1:num_landmarks
        est_x = lm(1, i);
        est_y = lm(2, i);
        
        gt_x = landmarks_gt(i, 1);
        gt_y = landmarks_gt(i, 2);
        
        plot(est_x, est_y, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
        
        plot([gt_x, est_x], [gt_y, est_y], 'r--', 'LineWidth', 1);
        
        error_dist = sqrt((est_x - gt_x)^2 + (est_y - gt_y)^2);
        
        mid_x = (gt_x + est_x) / 2;
        mid_y = (gt_y + est_y) / 2;
        text(mid_x, mid_y, sprintf('L%d\n%.2fm', i, error_dist), ...
             'FontSize', 8, 'Color', 'blue', 'BackgroundColor', 'none');
    end
    
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('FastSLAM Landmark Map: Groundtruth vs Estimated', 'FontSize', 14);
    
    legend('Groundtruth Landmarks', 'Estimated Landmarks', ...
           'Error Lines', 'Location', 'best');
    
    hold off;
    
    squared_errors = 0;
    
    for i = 1:num_landmarks
        est_x = lm(1, i);
        est_y = lm(2, i);
        gt_x = landmarks_gt(i, 1);
        gt_y = landmarks_gt(i, 2);
        error_dist = sqrt((est_x - gt_x)^2 + (est_y - gt_y)^2);
        squared_errors = squared_errors + error_dist^2;
    end
    
    rmse = sqrt(squared_errors / num_landmarks);
    fprintf('Landmark RMSE (final map): %.3f m\n', rmse);
end