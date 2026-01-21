function gui_resetSelection(fig)
    gui = guidata(fig);

    % Stop playback if running
    if isfield(gui,'timer') && isvalid(gui.timer) && strcmp(gui.timer.Running,'on')
        stop(gui.timer);
        gui.playBtn.Text = 'Play';
    end

    % Update IDs from dropdowns
    gui.datasetId = sscanf(gui.datasetDD.Value, 'Dataset%d');
    gui.robotId   = sscanf(gui.robotDD.Value,   'Robot%d');

    % Force reload on next step
    gui.data = [];
    gui.est  = [];
    gui.k    = 1;
    gui.estHist = [];

    % Invalidate plot handles so gui_updatePlot will re-init safely
    handleFields = {'hLM','hGT','hGTcur','hEst','hEstCur','hMap','hCov','hLmCov'};
    for i = 1:numel(handleFields)
        if isfield(gui, handleFields{i})
            gui.(handleFields{i}) = [];
        end
    end

    cla(gui.ax);
    guidata(fig, gui);
end
