function est = resample_particles(est)
%RESAMPLE_PARTICLES Systematic resampling

    N = est.N;

    w = [est.particles.w]';
    sw = sum(w);

    if sw <= 0 || ~isfinite(sw)
        for i = 1:N
            est.particles(i).w = 1/N;
        end
        return;
    end

    % Normalize
    w = w / sw;

    % CDF
    cdf = cumsum(w);

    % Systematic positions
    u0 = rand / N;
    u  = u0 + (0:N-1)'/N;

    % Index selection (single pass)
    idx = zeros(N,1);
    j = 1;
    for i = 1:N
        while u(i) > cdf(j)
            j = j + 1;
        end
        idx(i) = j;
    end

    % Resample particles
    est.particles = est.particles(idx);

    % Reset weights
    for i = 1:N
        est.particles(i).w = 1/N;
    end
end
