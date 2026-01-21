function state = get_state(est)

    if isfield(est,'mu') && isfield(est,'Sigma')
        state.mu = est.mu(1:3);
        state.Sigma = est.Sigma(1:3,1:3);
        return;

    elseif isfield(est,'x') && isfield(est,'P')
        state.mu = est.x(1:3);
        state.Sigma = est.P(1:3,1:3);
        return;

    elseif isfield(est,'particles') && ~isempty(est.particles)
        % ---- FastSLAM particle set ----
        N = numel(est.particles);

        X = zeros(3,N);
        w = zeros(1,N);
        for i = 1:N
            X(:,i) = est.particles(i).x(:);
            w(i)   = est.particles(i).w;
        end

        % normalize weights (robust)
        sw = sum(w);
        if sw <= 0 || ~isfinite(sw)
            w = ones(1,N)/N;
        else
            w = w / sw;
        end

        % weighted mean of x,y
        mu_xy = X(1:2,:) * w(:);

        % circular mean of theta
        th = X(3,:);
        c = sum(w .* cos(th));
        s = sum(w .* sin(th));
        mu_th = atan2(s, c);

        mu = [mu_xy; mu_th];

        % weighted covariance
        dX = X - mu;
        dX(3,:) = wrapToPi(dX(3,:));

        Sigma = zeros(3,3);
        for i = 1:N
            Sigma = Sigma + w(i) * (dX(:,i) * dX(:,i).');
        end

        state.mu = mu;
        state.Sigma = Sigma;
        return;

    else
        error('get_state: Estimator has neither (mu,Sigma), (x,P), nor particles.');
    end
end
