function est = fastslam_init(data, params)
%FASTSLAM_INIT Initialize FastSLAM (1.0) estimator struct.

    if nargin < 2 || isempty(params), params = struct(); end

    if ~isfield(params,'N'), params.N = 500; end
    if ~isfield(params,'R'), params.R = diag([0.15^2, (5*pi/180)^2]); end
    if ~isfield(params,'Q'), params.Q = diag([0.02^2, 0.02^2, (2*pi/180)^2]); end
    if ~isfield(params,'resample_thresh'), params.resample_thresh = 0.5; end
    if ~isfield(params,'initPose'), params.initPose = [0;0;0]; end

    % number of landmarks
    if isfield(data,'num_landmarks') && ~isempty(data.num_landmarks)
        nL = data.num_landmarks;
    elseif isfield(data,'landmarks_gt') && ~isempty(data.landmarks_gt)
        nL = size(data.landmarks_gt,1);
    else
        nL = 15;
    end

    N = params.N;

    particles = repmat(struct( ...
        'x', params.initPose(:), ...
        'w', 1/N, ...
        'lm', zeros(2,nL), ...
        'P', zeros(2,2,nL), ...
        'seen', false(1,nL) ...
    ), N, 1);

    % Large, finite initial covariance for all landmarks (ignored until seen=true)
    P0 = 1e6 * eye(2);

    for i = 1:N
        particles(i).lm = zeros(2,nL);
        particles(i).P  = repmat(P0, 1, 1, nL);
    end

    est.algo = 'FastSLAM';
    est.params = params;
    est.N = N;
    est.particles = particles;

    if isfield(data,'barcodes')
        est.barcodes = data.barcodes(:);
    else
        est.barcodes = [];
    end
    est.nL = nL;

    fprintf('FastSLAM initialized:\n');
    fprintf('  N = %d\n', N);
    fprintf('  sigma_r = %.3f m, sigma_b = %.2f deg\n', ...
        sqrt(params.R(1,1)), sqrt(params.R(2,2))*180/pi);
    fprintf('  Q diag = [%.6g %.6g %.6g]\n', params.Q(1,1), params.Q(2,2), params.Q(3,3));
end
