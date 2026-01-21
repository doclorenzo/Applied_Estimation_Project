function [mu, Sigma] = ekf_update(mu, Sigma, r, b, lm_index)

% Robot pose
x_r = mu(1);
y_r = mu(2);
theta = mu(3);

% Landmark position in state vector
lm_x = mu(3 + 2*lm_index - 1);
lm_y = mu(3 + 2*lm_index);

% Expected measurement (predicted)
dx = lm_x - x_r;
dy = lm_y - y_r;
q = dx^2 + dy^2;

z_hat = [sqrt(q);
         atan2(dy, dx) - theta];

R=diag([0.15^2, (5*pi/180)^2]);

% Jacobian 
sqrt_q = sqrt(q);
H = zeros(2, length(mu));
H(1,1) = -dx / sqrt_q;
H(1,2) = -dy / sqrt_q;
H(1,3) = 0;
H(1, 3+2*lm_index-1) = dx / sqrt_q;
H(1, 3+2*lm_index)   = dy / sqrt_q;

H(2,1) = dy / q;
H(2,2) = -dx / q;
H(2,3) = -1;
H(2, 3+2*lm_index-1) = -dy / q;
H(2, 3+2*lm_index)   = dx / q;

% Innovation
z = [r; b];
y = z - z_hat;
y(2) = wrapToPi(y(2)); 

S = H * Sigma * H' + R;

% Kalman gain
K = Sigma * H' / S;

% Update state
mu = mu + K * y;

% Update covariance
Sigma = (eye(length(mu)) - K * H) * Sigma;

end
