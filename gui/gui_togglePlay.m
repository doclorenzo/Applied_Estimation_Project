function gui_togglePlay(fig)

    gui = guidata(fig);

    if strcmp(gui.timer.Running,'off')
        start(gui.timer);
        gui.playBtn.Text = 'Pause';
    else
        stop(gui.timer);
        gui.playBtn.Text = 'Play';
    end

    guidata(fig, gui);
end
