function est = init_estimator(data, robotId, algoName)

    if nargin < 3 || isempty(algoName)
        algoName = 'EKF-SLAM';
    end

    est.algo = algoName;

    switch algoName
        case 'EKF-SLAM'
            num_landmarks = 15;

            if isfield(data,'truePose')
                x0 = data.truePose{robotId}(1,:)';
            else
                x0 = data.gt(1,:)';
            end

            est.mu = zeros(3 + 2*num_landmarks, 1);
            est.mu(1:3) = x0;

            est.Sigma = zeros(length(est.mu));
            est.Sigma(1:3,1:3) = 1e-3 * eye(3);
            est.Sigma(4:end,4:end) = 1e4 * eye(2*num_landmarks);

            est.seen = zeros(num_landmarks,1);

            if isfield(data,'barcodes')
                est.barcode_list = data.barcodes(:);
            else
                error('data.barcodes missing. Add gui.data.barcodes = rawData.Barcodes(:,2) in gui_loadDataset.');
            end

        case 'FastSLAM'
            % ---- init pose ----
            if isfield(data,'truePose')
                x0 = data.truePose{robotId}(1,1:3)';  % [x y th]
            elseif isfield(data,'gt')
                x0 = data.gt(1,1:3)';                % [x y th]
            else
                x0 = [0;0;0];
            end

            % ---- FastSLAM params ----
            params = struct();
            params.N = 500;

            % KEY FIX: don't kill bearing information
            % Start at 10 deg; once stable, try 7 deg then 5 deg.
            params.R = diag([0.15^2, (5*pi/180)^2]);

            % Motion noise (per-second covariance on pose increment; your predict scales by dt)
            params.Q = diag([0.02^2, 0.02^2, (2*pi/180)^2]);

            params.resample_thresh = 0.5;
            params.initPose = x0;
            params.k_w = 1.05;
            % ---- Provide FastSLAM init with the structure it expects ----
            num_landmarks = 15;

            if ~isfield(data,'barcodes')
                error('data.barcodes missing. Add gui.data.barcodes = rawData.Barcodes(:,2) in gui_loadDataset.');
            end

            fs_data = struct();
            fs_data.barcodes = data.barcodes(:);
            fs_data.num_landmarks = num_landmarks;

            est = fastslam_init(fs_data, params);

            % ---- O(1) barcode->landmark index map ----
            barcodes = fs_data.barcodes;
            maxBC = max(barcodes);
            bc2lm = zeros(maxBC, 1, 'int16');

            for subj = 6:numel(barcodes)
                bc = barcodes(subj);
                if bc > 0 && bc <= maxBC
                    bc2lm(bc) = int16(subj - 5); % landmarks 1..15
                end
            end

            est.bc2lm = bc2lm;
            est.maxBC = maxBC;

        otherwise
            error('Unknown algorithm: %s', algoName);
    end
end
