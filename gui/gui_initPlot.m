function gui = gui_initPlot(gui)

    ax = gui.ax;
    cla(ax);
    hold(ax,'on');

    % Lock view
    if isfield(gui,'plotBounds') && ~isempty(gui.plotBounds)
        xlim(ax, gui.plotBounds.xlim);
        ylim(ax, gui.plotBounds.ylim);
    end
    axis(ax,'equal');
    grid(ax,'on');

    % True landmarks (static)
    if isfield(gui.data,'landmarks_gt') && ~isempty(gui.data.landmarks_gt)
        gui.hLM = plot(ax, gui.data.landmarks_gt(:,1), gui.data.landmarks_gt(:,2), 'k.', 'MarkerSize', 10);
    else
        gui.hLM = plot(ax, NaN, NaN, 'k.');
    end

    % GT path + current GT
    gui.hGT    = plot(ax, NaN, NaN, 'g',  'LineWidth', 2.5);
    gui.hGTcur = plot(ax, NaN, NaN, 'go', 'MarkerSize', 10, 'LineWidth', 2.5);

    % Estimated path + current est
    gui.hEst    = plot(ax, NaN, NaN, 'b',  'LineWidth', 2.0);
    gui.hEstCur = plot(ax, NaN, NaN, 'bo', 'MarkerSize', 10, 'LineWidth', 2.5);

    % Estimated landmarks as a single object (fast)
    gui.hMap = plot(ax, NaN, NaN, 'rx', 'LineWidth', 2);

    % Robot covariance ellipse as a single line
    gui.hCov = plot(ax, NaN, NaN, '-', 'LineWidth', 1.2);
    set(gui.hCov, 'Visible', 'off');

    % --- Landmark covariance ellipse pool (created once) ---
    gui.hLmCov = gobjects(0);

    maxEllipses = 30; % safe upper bound (MRCLAM ~15 landmarks)
    gui.hLmCov = gobjects(maxEllipses,1);
    for i = 1:maxEllipses
        gui.hLmCov(i) = plot(ax, NaN, NaN, '-', 'LineWidth', 1.0);
        set(gui.hLmCov(i), 'Visible', 'off');
    end

    gui.hParticles = plot(ax, NaN, NaN, '.', 'MarkerSize', 6);

    hold(ax,'off');
end
