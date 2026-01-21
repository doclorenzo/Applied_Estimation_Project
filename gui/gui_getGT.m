function [gt_k, gt_cur, kPlot] = gui_getGT(gui)

    if isfield(gui.data,'gt_raw') && ~isempty(gui.data.gt_raw)
        kPlot  = min(gui.k, size(gui.data.gt_raw,1));
        gt_k   = gui.data.gt_raw(1:kPlot,1:2);
        gt_cur = gui.data.gt_raw(kPlot,1:2);
        return;
    end

    r = gui.robotId;
    if isfield(gui.data,'truePose') && ~isempty(gui.data.truePose)
        kPlot  = min(gui.k, size(gui.data.truePose{r},1));
        gt_k   = gui.data.truePose{r}(1:kPlot,1:2);
        gt_cur = gui.data.truePose{r}(kPlot,1:2);
        return;
    end

    kPlot  = min(gui.k, size(gui.data.gt,1));
    gt_k   = gui.data.gt(1:kPlot,1:2);
    gt_cur = gui.data.gt(kPlot,1:2);
end
