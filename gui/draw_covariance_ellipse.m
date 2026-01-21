function h = draw_covariance_ellipse(ax, mu, Sigma, conf, varargin)
%DRAW_COVARIANCE_ELLIPSE Draw a 2D covariance ellipse.
%   h = draw_covariance_ellipse(ax, mu, Sigma, conf)
%   mu:    [2x1] mean
%   Sigma: [2x2] covariance
%   conf:  confidence level in (0,1), e.g. 0.95

    p = inputParser;
    addParameter(p, 'NumPoints', 60);
    parse(p, varargin{:});
    nPts = p.Results.NumPoints;

    mu = mu(:);
    if numel(mu) ~= 2 || ~all(size(Sigma) == [2 2])
        error('mu must be 2x1 and Sigma must be 2x2');
    end

    Sigma = (Sigma + Sigma')/2;
    [V,D] = eig(Sigma);
    d = max(diag(D), 0);
    D = diag(d);

    s = chi2inv_2dof(conf);

    t = linspace(0, 2*pi, nPts);
    unit = [cos(t); sin(t)];
    A = V * sqrt(D * s);
    pts = mu + A * unit;

    h = plot(ax, pts(1,:), pts(2,:), 'LineWidth', 1.2);
end

function s = chi2inv_2dof(conf)
%CHI2INV_2DOF Approx/lookup for chi-square inverse CDF with 2 DOF.

    % Fast lookup for typical values:
    if abs(conf - 0.90) < 1e-12, s = 4.605; return; end
    if abs(conf - 0.95) < 1e-12, s = 5.991; return; end
    if abs(conf - 0.99) < 1e-12, s = 9.210; return; end
    if abs(conf - 0.9973) < 1e-12, s = 11.829; return; end % ~3-sigma in 2D

    z = sqrt(2) * erfinv(2*conf - 1); % ~N(0,1) quantile
    k = 2;
    s = k * (1 - 2/(9*k) + z*sqrt(2/(9*k)))^3;
    s = max(s, 0);
end
