function gui = gui_updatePlot(gui)

    if ~isfield(gui,'hGT') || isempty(gui.hGT) || ~isvalid(gui.hGT)
        gui = gui_initPlot(gui);
    end

    % Ground Truth
    [gt_k, gt_cur, kPlot] = gui_getGT(gui);

    stride = max(1, floor(kPlot/2000));
    idx = 1:stride:kPlot;

    set(gui.hGT,    'XData', gt_k(idx,1), 'YData', gt_k(idx,2));
    set(gui.hGTcur, 'XData', gt_cur(1),   'YData', gt_cur(2));

    isFast = isfield(gui,'est') && ~isempty(gui.est) && ...
             isfield(gui.est,'algo') && strcmp(gui.est.algo,'FastSLAM') && ...
             isfield(gui.est,'particles') && ~isempty(gui.est.particles);

    % Estimated trajectory
    if isFast
        if isfield(gui,'estHist') && ~isempty(gui.estHist)
            kE = min(size(gui.estHist,1), gui.k);
            strideE = max(1, floor(kE/2000));
            idE = 1:strideE:kE;
            set(gui.hEst, 'XData', gui.estHist(idE,1), 'YData', gui.estHist(idE,2));
        else
            set(gui.hEst, 'XData', NaN, 'YData', NaN);
        end
    else
        % EKF etc: keep your existing history
        if isfield(gui,'estHist') && ~isempty(gui.estHist)
            kE = min(size(gui.estHist,1), gui.k);
            strideE = max(1, floor(kE/2000));
            idE = 1:strideE:kE;
            set(gui.hEst, 'XData', gui.estHist(idE,1), 'YData', gui.estHist(idE,2));
        else
            set(gui.hEst, 'XData', NaN, 'YData', NaN);
        end
    end

    % Current estimated pose + robot covariance
    if ~isempty(gui.est)

        if isFast
            [mu_xyth, Sig_xy] = fastslam_pose_mean_and_cov(gui.est);

            set(gui.hEstCur, 'XData', mu_xyth(1), 'YData', mu_xyth(2));

            [ex, ey] = covarianceEllipsePoints(mu_xyth(1:2), Sig_xy, 0.95, 40);
            set(gui.hCov, 'XData', ex, 'YData', ey, 'Visible', 'on');

        else
            %EKF-SLAM 
            st = get_state(gui.est);
            set(gui.hEstCur, 'XData', st.mu(1), 'YData', st.mu(2));

            Sigma_xy = st.Sigma(1:2,1:2);
            [ex, ey] = covarianceEllipsePoints(st.mu(1:2), Sigma_xy, 0.95, 40);
            set(gui.hCov, 'XData', ex, 'YData', ey, 'Visible', 'on');
        end

    else
        set(gui.hEstCur, 'XData', NaN, 'YData', NaN);
        set(gui.hCov, 'Visible', 'off');
    end

    % ---------- Particle cloud (FastSLAM only, throttled) ----------
    PARTICLE_STRIDE = 5;     % update every 5 frames (tune 2..10)
    MAX_PARTICLES_DRAW = 5000;

    if isFast
        if ~isfield(gui,'hParticles') || isempty(gui.hParticles) || ~isvalid(gui.hParticles)
            hold(gui.ax,'on');
            gui.hParticles = plot(gui.ax, NaN, NaN, '.', 'MarkerSize', 8);
            set(gui.hParticles, 'Color', [0.6 0.6 0.6]);
            hold(gui.ax,'off');
        end

        if mod(gui.k, PARTICLE_STRIDE) == 0
            Np = numel(gui.est.particles);
            K  = min(MAX_PARTICLES_DRAW, Np);

            % systematic resampling for display (fast + low-variance)
            w = zeros(Np,1);
            for i = 1:Np, w(i) = gui.est.particles(i).w; end
            sw = sum(w);
            if sw <= 0 || ~isfinite(sw)
                w = ones(Np,1)/Np;
            else
                w = w / sw;
            end
            cdf = cumsum(w);

            sel = systematic_sample(cdf, K);

            Pxy = zeros(K,2);
            for kk = 1:K
                Pxy(kk,:) = gui.est.particles(sel(kk)).x(1:2).';
            end

            set(gui.hParticles, 'XData', Pxy(:,1), 'YData', Pxy(:,2), 'Visible', 'on');
        end
    else
        if isfield(gui,'hParticles') && ~isempty(gui.hParticles) && isvalid(gui.hParticles)
            set(gui.hParticles, 'Visible', 'off');
        end
    end

    %Estimated landmarks
    if ~isempty(gui.est)
        map = get_map(gui.est);

        if isfield(map,'landmarks') && ~isempty(map.landmarks)
            n = numel(map.landmarks);
            M = NaN(n,2);
            for i = 1:n
                mu = map.landmarks(i).mu;
                if numel(mu) >= 2
                    M(i,:) = mu(1:2).';
                end
            end
            M = M(all(isfinite(M),2),:);
        else
            M = [];
        end

        if ~isempty(M)
            set(gui.hMap, 'XData', M(:,1), 'YData', M(:,2));
        else
            set(gui.hMap, 'XData', NaN, 'YData', NaN);
        end

        %Landmark covariance ellipses
        ELLIPSE_STRIDE = 10;
        ELLIPSE_PTS    = 25;

        doLmEllipses = (mod(gui.k, ELLIPSE_STRIDE) == 0);

        if doLmEllipses && isfield(gui,'hLmCov') && ~isempty(gui.hLmCov) ...
                && isfield(map,'landmarks') && ~isempty(map.landmarks)

            used = 0;
            for i = 1:numel(map.landmarks)

                if ~isfield(map.landmarks(i),'Sigma') || isempty(map.landmarks(i).Sigma)
                    continue;
                end

                Sigma = map.landmarks(i).Sigma;
                mu    = map.landmarks(i).mu;

                if numel(mu) < 2 || ~all(size(Sigma) == [2 2])
                    continue;
                end

                used = used + 1;
                if used > numel(gui.hLmCov)
                    break;
                end

                [ex, ey] = covarianceEllipsePoints(mu(1:2), Sigma, 0.95, ELLIPSE_PTS);
                set(gui.hLmCov(used), 'XData', ex, 'YData', ey, 'Visible', 'on');
            end

            for j = used+1:numel(gui.hLmCov)
                set(gui.hLmCov(j), 'Visible', 'off');
            end

        end

    else
        set(gui.hMap, 'XData', NaN, 'YData', NaN);
        if isfield(gui,'hLmCov') && ~isempty(gui.hLmCov)
            for j = 1:numel(gui.hLmCov)
                set(gui.hLmCov(j), 'Visible', 'off');
            end
        end
    end

    drawnow limitrate nocallbacks;
end

%helpers

function [mu_xyth, Sig_xy] = fastslam_pose_mean_and_cov(est)
% mean pose and XY covariance from particles

    Np = numel(est.particles);
    X  = zeros(Np,3);

    for i = 1:Np
        X(i,:) = est.particles(i).x(:).';
    end

    % Mean position
    mx = mean(X(:,1));
    my = mean(X(:,2));

    %Circular mean for theta
    mth = atan2(mean(sin(X(:,3))), mean(cos(X(:,3))));

    mu_xyth = [mx; my; mth];

    % XY covariance (sample covariance
    dx = X(:,1) - mx;
    dy = X(:,2) - my;

    Sig_xy = [mean(dx.^2), mean(dx.*dy);
              mean(dx.*dy), mean(dy.^2)];

    % tiny jitter for numerical stability
    Sig_xy = Sig_xy + 1e-12*eye(2);
end


function sel = systematic_sample(cdf, K)
% Systematic sampling indices from a CDF (cdf(end) assumed ~1)

    u0 = rand() / K;
    u = u0 + (0:K-1)'/K;

    sel = zeros(K,1);
    j = 1;
    for k = 1:K
        while j < numel(cdf) && cdf(j) < u(k)
            j = j + 1;
        end
        sel(k) = j;
    end
end
