function map = get_map(est)

    map.landmarks = [];

    if isfield(est,'mu') && isfield(est,'Sigma')

        num_landmarks = (length(est.mu) - 3) / 2;
        idx = 0;

        for lm = 1:num_landmarks
            if isfield(est,'seen') && est.seen(lm) == 0
                continue;
            end

            xi = 3 + 2*lm - 1;
            yi = 3 + 2*lm;

            idx = idx + 1;
            map.landmarks(idx).mu    = [est.mu(xi); est.mu(yi)];
            map.landmarks(idx).Sigma = est.Sigma([xi yi],[xi yi]);
        end
        return;
    end

    % ---------- FastSLAM case ----------
    if isfield(est,'particles') && ~isempty(est.particles)

        % choose best particle (max weight)
        w = [est.particles.w];
        [~, best] = max(w);
        p = est.particles(best);

        if ~isfield(p,'lm') || isempty(p.lm)
            return;
        end

        nL = size(p.lm,2);
        idx = 0;

        for lm = 1:nL
            if isfield(p,'seen') && ~p.seen(lm)
                continue;
            end

            muL = p.lm(:,lm);
            if any(~isfinite(muL))
                continue;
            end

            idx = idx + 1;
            map.landmarks(idx).mu = muL;

            if isfield(p,'P') && ~isempty(p.P)
                SL = p.P(:,:,lm);
                if all(size(SL) == [2 2]) && all(isfinite(SL(:)))
                    map.landmarks(idx).Sigma = SL;
                else
                    map.landmarks(idx).Sigma = [];
                end
            else
                map.landmarks(idx).Sigma = [];
            end
        end

        return;
    end

    error('get_map: Estimator has neither EKF (mu,Sigma) nor FastSLAM particles.');
end
