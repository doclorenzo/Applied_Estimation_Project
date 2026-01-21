function [x, y] = covarianceEllipsePoints(mu, Sigma, conf, nPts)
% Returns ellipse points for 2D covariance at confidence conf (0..1).

    if nargin < 4, nPts = 60; end
    mu = mu(:);
    Sigma = (Sigma + Sigma')/2;

    [V,D] = eig(Sigma);
    d = max(diag(D),0);
    D = diag(d);

    if abs(conf-0.95) < 1e-12
        s = 5.991;
    elseif abs(conf-0.99) < 1e-12
        s = 9.210;
    else
        z = sqrt(2) * erfinv(2*conf - 1);
        s = 2 * (1 - 2/18 + z*sqrt(2/18))^3;
        s = max(s,0);
    end

    t = linspace(0, 2*pi, nPts);
    unit = [cos(t); sin(t)];
    A = V * sqrt(D*s);
    pts = mu + A*unit;

    x = pts(1,:);
    y = pts(2,:);
end
