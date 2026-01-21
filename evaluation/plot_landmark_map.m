function plot_landmark_map(mu, landmarks_gt, seen, num_landmarks)
    
    figure('Name', 'Landmark Map', 'Position', [100, 100, 900, 800]);
    hold on;
    grid on;
    axis equal;
    
    %  groundtruth landmarks
    plot(landmarks_gt(:,1), landmarks_gt(:,2), 'go', ...
         'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', [0.7 1 0.7]);
    
    %  estimated landmarks and error lines
    for i = 1:num_landmarks
        if seen(i)
            est_x = mu(3 + 2*i - 1);
            est_y = mu(3 + 2*i);
            
            gt_x = landmarks_gt(i, 1);
            gt_y = landmarks_gt(i, 2);
            
            plot(est_x, est_y, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
            
            plot([gt_x, est_x], [gt_y, est_y], 'r--', 'LineWidth', 1);
            
            error_dist = sqrt((est_x - gt_x)^2 + (est_y - gt_y)^2);
            
            mid_x = (gt_x + est_x) / 2;
            mid_y = (gt_y + est_y) / 2;
            text(mid_x, mid_y, sprintf('L%d\n%.2fm', i, error_dist), ...
                 'FontSize', 8, 'Color', 'blue', 'BackgroundColor', 'white');
        end
    end
    
    robot_x = mu(1);
    robot_y = mu(2);
    robot_theta = mu(3);
    
    plot(robot_x, robot_y, 'bs', 'MarkerSize', 10, ...
         'LineWidth', 2, 'MarkerFaceColor', 'blue');
    
    arrow_length = 1.0;
    quiver(robot_x, robot_y, ...
           arrow_length * cos(robot_theta), ...
           arrow_length * sin(robot_theta), ...
           0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('Landmark Map: Groundtruth vs Estimated', 'FontSize', 14);
    
    legend('Groundtruth Landmarks', 'Estimated Landmarks', ...
           'Error Lines', 'Location', 'best');
    
    hold off;
    num_observed = sum(seen);
    total_error = 0;
    
    fprintf('\n--- Landmark Estimation Summary ---\n');
    fprintf('Observed landmarks: %d / %d\n', num_observed, num_landmarks);
    
    for i = 1:num_landmarks
        if seen(i)
            est_x = mu(3 + 2*i - 1);
            est_y = mu(3 + 2*i);
            gt_x = landmarks_gt(i, 1);
            gt_y = landmarks_gt(i, 2);
            error_dist = sqrt((est_x - gt_x)^2 + (est_y - gt_y)^2);
            total_error = total_error + error_dist;
            fprintf('Landmark %2d: Error = %.3f m\n', i, error_dist);
        end
    end
    
    if num_observed > 0
        avg_error = total_error / num_observed;
        fprintf('Average landmark error: %.3f m\n', avg_error);
    end
end