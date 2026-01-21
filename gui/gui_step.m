function gui_step(fig)

    gui = guidata(fig);

    if isempty(gui.data)
        gui = gui_loadDataset(gui);
        gui.estHist = []; % reset history on load/reset
        guidata(fig, gui);
    end

    r = gui.robotId;

    if isfield(gui.data,'time') && isfield(gui.data,'odometry') && isfield(gui.data,'measurements')
        tvec = gui.data.time;
        N = length(tvec);
        if gui.k > N
            stop(gui.timer); gui.playBtn.Text = 'Play';
            guidata(fig, gui); return;
        end
        vw = gui.data.odometry{r}(gui.k,:)';
        z  = gui.data.measurements{r}{gui.k};
        t  = tvec(gui.k);
        t_prev = tvec(max(gui.k-1,1));
    else
        tvec = gui.data.t;
        N = length(tvec);
        if gui.k > N
            stop(gui.timer); gui.playBtn.Text = 'Play';
            guidata(fig, gui); return;
        end
        vw = gui.data.odom(gui.k,:)';
        z  = gui.data.obs{gui.k};
        t  = tvec(gui.k);
        t_prev = tvec(max(gui.k-1,1));
    end

    if any(isnan(vw))
        vw(isnan(vw)) = 0;
    end

    v  = vw(1);
    w  = vw(2);
    dt = max(0, t - t_prev);

    % --- Step estimator ---
    gui.est = step_estimator(gui.est, v, w, dt, z, gui.data);

    % --- Store estimated trajectory history ---
    if ~isfield(gui,'estHist') || isempty(gui.estHist)
        % store [x y theta] for convenience (theta optional)
        gui.estHist = NaN(gui.k,3);
    elseif size(gui.estHist,1) < gui.k
        gui.estHist(gui.k,1:3) = NaN;
    end

    isFast = isfield(gui,'est') && ~isempty(gui.est) && ...
             isfield(gui.est,'algo') && strcmp(gui.est.algo,'FastSLAM') && ...
             isfield(gui.est,'particles') && ~isempty(gui.est.particles);

    if isFast
        [mu_xyth] = fastslam_mean_pose(gui.est);
        gui.estHist(gui.k,1:3) = mu_xyth(:).';
    else
        st = get_state(gui.est);
        gui.estHist(gui.k,1:3) = st.mu(1:3).';
    end

    % --- Plot ---
    if mod(gui.k,5)==0
        gui = gui_updatePlot(gui);
    end

    gui.k = gui.k + 1;
    guidata(fig, gui);
end

% ========================= helpers =========================

function mu_xyth = fastslam_mean_pose(est)

    Np = numel(est.particles);
    w  = zeros(Np,1);
    X  = zeros(Np,3);

    for i = 1:Np
        X(i,:) = est.particles(i).x(:).';
    end

    w = ones(Np,1)/Np;
    sw = sum(w);
    w = w / sw;
    
    
    mx = sum(w .* X(:,1));
    my = sum(w .* X(:,2));

    c = sum(w .* cos(X(:,3)));
    s = sum(w .* sin(X(:,3)));
    mth = atan2(s, c);

    mu_xyth = [mx; my; mth];
end
