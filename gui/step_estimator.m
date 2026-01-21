function est = step_estimator(est, v, w, dt, z, ~)

    switch est.algo
        case 'EKF-SLAM'
            % est = ekf_step_from_measurements(est, v, w, dt, z);
            [est.mu, est.Sigma] = ekf_predict(est.mu, est.Sigma, v, w, dt);
            barcode_list = est.barcode_list;

            for i = 1:size(z,1)
                barcode_num = z(i,1);
                r = z(i,2);
                b = z(i,3);
        
                lm_index_full = find(barcode_list == barcode_num, 1);
        
                % MRCLAM: first 5 barcodes are robots, landmarks start at 6
                if isempty(lm_index_full) || lm_index_full < 6
                    continue;
                end
        
                lm_index = lm_index_full - 5; % 1..15
        
                % Init landmark if first time seen
                if est.seen(lm_index) == 0
                    x_r = est.mu(1);
                    y_r = est.mu(2);
                    theta = est.mu(3);
        
                    est.mu(3 + 2*lm_index - 1) = x_r + r*cos(theta + b);
                    est.mu(3 + 2*lm_index)     = y_r + r*sin(theta + b);
        
                    est.seen(lm_index) = 1;
                end
        
                % Update
                [est.mu, est.Sigma] = ekf_update(est.mu, est.Sigma, r, b, lm_index);
            end

        case 'FastSLAM'
            w_used = est.params.k_w .* w;
            est = fastslam_predict(est, v, w_used, dt);
            est = fastslam_update(est, z);

        otherwise
            error('Unknown algorithm: %s', est.algo);
    end
end
