function slam_gui()
    %% Main window
    fig = uifigure('Name','SLAM Visualizer', ...
        'Position',[100 100 1050 720], ...
        'CloseRequestFcn',@(src,~)onClose(src));

    %% Axes
    ax = uiaxes(fig,'Position',[50 100 650 580]);
    axis(ax,'equal'); grid(ax,'on');
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');

    %% Controls

    % Dataset selector (1..9)
    datasetDD = uidropdown(fig, ...
        'Items', arrayfun(@(i)sprintf('Dataset%d',i), 1:9, 'UniformOutput', false), ...
        'Position',[750 640 200 30], ...
        'Value','Dataset1');

    % Robot selector (1..5)
    robotDD = uidropdown(fig, ...
        'Items', arrayfun(@(i)sprintf('Robot%d',i), 1:5, 'UniformOutput', false), ...
        'Position',[750 600 200 30], ...
        'Value','Robot1');

    % Algorithm selector
    algoDD = uidropdown(fig, ...
        'Items',{'EKF-SLAM','FastSLAM'}, ...
        'Position',[750 560 200 30], ...
        'Value','EKF-SLAM');

    playBtn = uibutton(fig,'Text','Play', ...
        'Position',[750 500 90 30]); 

    reloadBtn = uibutton(fig,'Text','Load/Reset', ...
        'Position',[750 300 200 30]);

    %% Shared GUI state
    gui.fig       = fig;
    gui.ax        = ax;

    gui.datasetDD = datasetDD;
    gui.robotDD   = robotDD;
    gui.algoDD    = algoDD;
    gui.playBtn   = playBtn;
    gui.reloadBtn = reloadBtn;

    gui.data      = [];
    gui.est       = [];
    gui.k         = 1;
    gui.estHist = [];

    gui.datasetId = 1;
    gui.robotId   = 1;

    %% Timer
    gui.timer = timer( ...
        'ExecutionMode','fixedRate', ...
        'Period', 0.001, ...
        'TimerFcn', @(~,~)gui_step(fig));

    %% Attach callbacks
    playBtn.ButtonPushedFcn       = @(~,~)gui_togglePlay(fig);

    datasetDD.ValueChangedFcn     = @(~,~)gui_resetSelection(fig);
    robotDD.ValueChangedFcn       = @(~,~)gui_resetSelection(fig);
    algoDD.ValueChangedFcn        = @(~,~)gui_resetSelection(fig);
    reloadBtn.ButtonPushedFcn     = @(~,~)gui_resetSelection(fig);

    guidata(fig, gui);

    function onClose(figHandle)
        gui = guidata(figHandle);
        if isfield(gui,'timer') && isvalid(gui.timer)
            stop(gui.timer);
            delete(gui.timer);
        end
        delete(figHandle);
    end
end
