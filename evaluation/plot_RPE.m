function plot_RPE(trans_error, delta)

    
    M = length(trans_error);
    step_vector = 1:M;
    
    max_plot_points = 10000;
    downsample_factor = max(1, floor(M / max_plot_points));
    
    step_plot = 1:length(step_vector(1:downsample_factor:end));
    error_plot = trans_error(1:downsample_factor:end);
    
    figure('Name', 'RPE Translational Error', 'Position', [100, 100, 1200, 500]);
    
    plot(step_plot, error_plot, 'b-', 'LineWidth', 0.5);
    xlim([0, length(step_plot)]);
    grid on;
    xlabel('Step Index', 'FontSize', 13);
    ylabel('Translational Error (m)', 'FontSize', 13);
    title(sprintf('RPE Translational Error (\\delta = %d steps)', delta), 'FontSize', 14);
    
    % Statistics
    rmse = sqrt(mean(trans_error.^2));
    fprintf('\nRMSE: %.4f m\n', rmse);
    fprintf('Mean: %.4f m\n', mean(trans_error));
    fprintf('Max: %.4f m\n', max(trans_error));
end