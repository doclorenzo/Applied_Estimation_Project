function est = fastslam_predict(est, v, w, dt)
%FASTSLAM_PREDICT.
% Uses per-second covariance Q, scaled by dt.
%
% REQUIREMENTS:
%  - est.particles(i).x is 3x1
%  - est.params.Q is 3x3
%  - wrapToPi available

    if dt <= 0 || isempty(est) || est.N <= 0
        return;
    end

    Q = est.params.Q;

    N = est.N;
    X = reshape([est.particles.x], 3, N);
    th = X(3,:);

    if abs(w) < 1e-9
        dx1 = (v*dt) .* cos(th);
        dx2 = (v*dt) .* sin(th);
        dx3 = (w*dt) .* ones(1,N);
    else
        th2 = th + w*dt;
        vw  = v / w;
        dx1 = vw .* (sin(th2) - sin(th));
        dx2 = vw .* (-cos(th2) + cos(th));
        dx3 = (w*dt) .* ones(1,N);
    end

    L = chol(Q*dt + 1e-12*eye(3), 'lower');
    noise = L * randn(3, N);

    X(1,:) = X(1,:) + dx1 + noise(1,:);
    X(2,:) = X(2,:) + dx2 + noise(2,:);
    X(3,:) = wrapToPi(X(3,:) + dx3 + noise(3,:));
    
    for i = 1:N
        est.particles(i).x = X(:,i);
    end
end
