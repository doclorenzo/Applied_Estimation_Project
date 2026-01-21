function est = fastslam_update(est, z)
%FASTSLAM_UPDATE
% z: Mx3 [barcode r b]

    if isempty(z)
        est = normalize_weights(est);
        return;
    end

    R     = est.params.R;
    bc2lm = est.bc2lm;
    maxBC = est.maxBC;

    % --- gating / robustness settings
    gate_soft    = 9.210;   % chi2inv(0.99,2)
    skip_penalty = 0.3;     % log-weight penalty when skipping an update
    loglik_floor = -80;     % clip log-likelihood to avoid crazy weights

    z_bc = z(:,1);
    z_r  = z(:,2);
    z_b  = z(:,3);

    % valid barcode range
    validBC = (z_bc > 0) & (z_bc <= maxBC);
    if ~any(validBC)
        est = normalize_weights(est);
        return;
    end

    z_bc = z_bc(validBC);
    z_r  = z_r(validBC);
    z_b  = z_b(validBC);

    % map barcode -> landmark index (0 means unknown)
    lm_idx = bc2lm(z_bc);
    validLm = (lm_idx ~= 0);

    if ~any(validLm)
        est = normalize_weights(est);
        return;
    end

    lm_idx = lm_idx(validLm);
    z_r    = z_r(validLm);
    z_b    = z_b(validLm);

    M = numel(lm_idx);
    N = est.N;

    % accumulate log weights
    logw = zeros(N,1);
    for i = 1:N
        logw(i) = log(est.particles(i).w + realmin);
    end

    % Outer loop: measurements, inner loop: particles
    for m = 1:M
        lm     = lm_idx(m);
        r_meas = z_r(m);
        b_meas = z_b(m);

        for i = 1:N
            p  = est.particles(i);
            xi = p.x;

            if any(~isfinite(xi))
                logw(i) = log(realmin);
                continue;
            end

            % initialize if unseen
            if ~p.seen(lm)
                [muL0, PL0] = init_landmark_from_measurement(xi, r_meas, b_meas, R);
                if all(isfinite(muL0)) && all(isfinite(PL0(:)))
                    p.lm(:,lm)    = muL0;
                    p.P(:,:,lm)   = PL0;
                    p.seen(lm)    = true;
                end
                est.particles(i) = p;
                continue; % no weight update on first sighting
            end

            muL = p.lm(:,lm);
            PL  = p.P(:,:,lm);

            % recover if NaN/Inf landmark state
            if any(~isfinite(muL)) || any(~isfinite(PL(:)))
                [muL0, PL0] = init_landmark_from_measurement(xi, r_meas, b_meas, R);
                if all(isfinite(muL0)) && all(isfinite(PL0(:)))
                    p.lm(:,lm)    = muL0;
                    p.P(:,:,lm)   = PL0;
                    p.seen(lm)    = true;
                end
                est.particles(i) = p;
                continue;
            end

            [muL_new, PL_new, loglik, nis] = ekf_landmark_update_nis(xi, muL, PL, r_meas, b_meas, R);

            % NIS gating (soft skip)
            if ~isfinite(nis) || nis > gate_soft
                logw(i) = logw(i) - skip_penalty;
                continue;
            end

            if any(~isfinite(muL_new)) || any(~isfinite(PL_new(:))) || ~isfinite(loglik)
                logw(i) = logw(i) - 1.0;
                continue;
            end

            p.lm(:,lm)   = muL_new;
            p.P(:,:,lm)  = PL_new;

            % clip likelihood contribution
            loglik = max(loglik, loglik_floor);
            logw(i) = logw(i) + loglik;

            est.particles(i) = p;
        end
    end

    % normalize weights 
    logw = logw - max(logw);
    w = exp(logw);
    w(~isfinite(w)) = 0;

    sw = sum(w);
    if sw <= realmin
        w = ones(N,1) / N;
    else
        w = w / sw;
    end

    for i = 1:N
        est.particles(i).w = w(i);
    end

    % resample based on Neff
    Neff = 1 / sum(w.^2);
    if Neff < est.params.resample_thresh * N
        est = resample_particles(est);
    end
end


function [muL, PL, loglik, nis] = ekf_landmark_update_nis(x, muL, PL, r_meas, b_meas, R)
% EKF landmark update + returns log-likelihood and NIS

    loglik = log(realmin);
    nis    = NaN;

    if any(~isfinite(x)) || any(~isfinite(muL)) || any(~isfinite(PL(:))) || any(~isfinite(R(:)))
        return;
    end

    xr = x(1);  yr = x(2);  th = x(3);
    dx = muL(1) - xr;
    dy = muL(2) - yr;

    q = dx*dx + dy*dy;
    if ~isfinite(q) || q < 1e-12
        nis = inf;
        return;
    end

    r_hat = sqrt(q);
    b_hat = wrapToPi(atan2(dy, dx) - th);

    nu = [r_meas - r_hat;
          wrapToPi(b_meas - b_hat)];

    H = [ dx/r_hat,  dy/r_hat;
         -dy/q,      dx/q];

    S = H*PL*H' + R;
    S = 0.5*(S + S');

    % mild stabilization (keep it lightweight)
    if any(~isfinite(S(:))) || rcond(S) < 1e-12
        S = S + 1e-9*eye(2);
    end

    % NIS (for gating)
    nis = nu' / S * nu;

    % log-likelihood
    loglik = gaussian_loglikelihood(nu, S);

    % standard EKF update (Joseph form)
    K   = (PL*H') / S;
    muL = muL + K*nu;

    I = eye(2);
    PL = (I - K*H)*PL*(I - K*H)' + K*R*K';
    PL = 0.5*(PL + PL');
end


function logp = gaussian_loglikelihood(nu, S)
% log N(nu; 0, S) computed stably via Cholesky
    d = numel(nu);

    [L,flag] = chol(S,'lower');
    if flag ~= 0
        logp = log(realmin);
        return;
    end

    y = L \ nu;
    maha = y' * y;
    logdet = 2 * sum(log(diag(L)));

    logp = -0.5*(d*log(2*pi) + logdet + maha);
end
