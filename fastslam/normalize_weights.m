function est = normalize_weights(est)
    w = [est.particles.w];
    s = sum(w);
    if s <= 0 || ~isfinite(s)
        for i = 1:est.N
            est.particles(i).w = 1/est.N;
        end
    else
        for i = 1:est.N
            est.particles(i).w = est.particles(i).w / s;
        end
    end
end
