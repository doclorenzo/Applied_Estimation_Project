function M = gui_getEstLandmarksXY(est)
% Returns Nx2 of landmark means if available, else []

    M = [];

    try
        map = get_map(est);
    catch
        return;
    end

    if ~isfield(map,'landmarks') || isempty(map.landmarks)
        return;
    end

    n = numel(map.landmarks);
    M = NaN(n,2);
    for i = 1:n
        mu = map.landmarks(i).mu;
        if numel(mu) >= 2
            M(i,:) = mu(1:2).';
        end
    end

    % remove NaNs
    M = M(all(isfinite(M),2),:);
end
