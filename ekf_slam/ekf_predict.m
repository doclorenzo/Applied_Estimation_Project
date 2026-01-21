function [mu, Sigma] = ekf_predict(mu, Sigma, v, w, dt)
    
    theta = mu(3); 
    
    alpha = [0.01, 0.01, 0.001, 0.1];
    sigma_v = alpha(1)*v^2 + alpha(2)*w^2;
    sigma_w = alpha(3)*v^2 + alpha(4)*w^2;
    
    V = [-cos(theta)*dt, 0;
         -sin(theta)*dt, 0;
          0,             dt];
    
    M = diag([sigma_v, sigma_w]);
    R = V * M * V';  
    
    G = eye(length(mu));
    G(1:3,1:3) = [1, 0, -v * sin(theta) * dt;
                  0, 1,  v * cos(theta) * dt;
                  0, 0, 1];
    
    % Predict new state
    mu(1) = mu(1) + v * cos(theta) * dt;
    mu(2) = mu(2) + v * sin(theta) * dt;
    mu(3) = mu(3) + w * dt;
    mu(3) = wrapToPi(mu(3));  
    
    R_full = blkdiag(R, zeros(length(mu)-3));
    Sigma = G * Sigma * G' + R_full;
end