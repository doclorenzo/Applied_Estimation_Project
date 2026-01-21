function [rpe_rmse, trans_error] = RPE(P, Q, idx_lenght, delta)

    N = idx_lenght;
    M = N - delta;
    trans_error = zeros(M, 1);
    
    new_P = cell(N, 1);
    new_Q = cell(N, 1);
    
    % Convert all poses to SE(2) matrices
    for i = 1:N
        x_gt = Q(i,1); y_gt = Q(i,2); th_gt = Q(i,3);
        new_Q{i} = [ cos(th_gt) -sin(th_gt) x_gt;
                     sin(th_gt)  cos(th_gt) y_gt;
                     0           0          1 ];
        
        mu = P(i, :);
        new_P{i} = [ cos(mu(3)) -sin(mu(3)) mu(1);
                     sin(mu(3))  cos(mu(3)) mu(2);
                     0           0          1 ];
    end 
    

    for i = 1:M

        P_rel = new_P{i} \ new_P{i + delta}; 
        Q_rel = new_Q{i} \ new_Q{i + delta};
        
        E = Q_rel \ P_rel;
        
        t = E(1:2, 3);
        trans_error(i) = norm(t);
    end
    
    % RMSE
    rpe_rmse = sqrt(mean(trans_error.^2));
end