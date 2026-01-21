function [muL, PL] = init_landmark_from_measurement(x, r, b, R)
%INIT_LANDMARK_FROM_MEASUREMENT
% Inputs:
%   x - robot pose [x; y; theta]
%   r - range measurement
%   b - bearing measurement (relative to robot heading)
%   R - measurement noise covariance [2x2]
%
% Outputs:
%   muL - landmark position [x; y]
%   PL  - landmark covariance [2x2]

    xr = x(1); yr = x(2); th = x(3);

    ang = wrapToPi(th + b);

    muL = [xr + r*cos(ang);
           yr + r*sin(ang)];

    G = [cos(ang), -r*sin(ang);
         sin(ang),  r*cos(ang)];

    PL = G * R * G';

    sig_xy = 0.25;             % meters
    sig_th = 8*pi/180;         % radians

    Jx = [1, 0, -r*sin(ang);
          0, 1,  r*cos(ang)];

    Px = diag([sig_xy^2, sig_xy^2, sig_th^2]);

    PL = PL + Jx * Px * Jx';

    PL = PL + (0.50^2) * eye(2);

    PL = 0.5*(PL + PL');
end