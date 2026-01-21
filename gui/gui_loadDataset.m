function gui = gui_loadDataset(gui)

    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);

    if isfield(gui,'datasetId') && ~isempty(gui.datasetId)
        datasetId = gui.datasetId;
    else
        datasetId = sscanf(gui.datasetDD.Value, 'Dataset%d');
    end

    if isfield(gui,'robotId') && ~isempty(gui.robotId)
        robotId = gui.robotId;
    elseif isfield(gui,'robotDD')
        robotId = sscanf(gui.robotDD.Value, 'Robot%d');
    else
        robotId = sscanf(gui.datasetDD.Value, 'Robot%d');
    end

    datasetId = max(1, min(9, datasetId));
    robotId   = max(1, min(5, robotId));
    gui.datasetId = datasetId;
    gui.robotId   = robotId;

    candidates = { ...
        fullfile(projectRoot, 'data', 'raw', sprintf('MRCLAM_Dataset%d', datasetId)), ...
        fullfile(projectRoot, 'data', sprintf('MRCLAM_Dataset%d', datasetId)), ...
        fullfile(projectRoot, sprintf('MRCLAM_Dataset%d', datasetId)) ...
    };

    datasetPath = '';
    for i = 1:numel(candidates)
        if isfolder(candidates{i})
            datasetPath = candidates{i};
            break;
        end
    end
    if isempty(datasetPath)
        error('Dataset folder not found. Tried:\n- %s\n- %s\n- %s', candidates{1}, candidates{2}, candidates{3});
    end

    rawData = loadMRCLAM_function(datasetPath);

    gui.data = struct();
    gui.data.barcodes = rawData.Barcodes(:,2);

    if isfield(rawData,'Landmark_Groundtruth')
        gui.data.landmarks_gt = rawData.Landmark_Groundtruth(:,2:3); % [x y] global
        gui.data.num_landmarks = size(rawData.Landmark_Groundtruth,1);
    else
        gui.data.landmarks_gt = [];
        gui.data.num_landmarks = 15;
    end

    gui.data.t_raw  = rawData.Robot(robotId).Groundtruth(:,1);
    gui.data.gt_raw = rawData.Robot(robotId).Groundtruth(:,2:4); % [x y theta] global

    useSampled = false;
    if exist('sampleMRCLAMdataSet','file') == 2
        try
            samplePeriod = 0.1;
            [dataSamp, ~] = sampleMRCLAMdataSet(rawData, samplePeriod);

            if isfield(dataSamp,'odometry') && isfield(dataSamp,'measurements') && isfield(dataSamp,'truePose') && isfield(dataSamp,'time')
                gui.data.odometry     = dataSamp.odometry;
                gui.data.measurements = dataSamp.measurements;
                gui.data.time         = dataSamp.time;
                useSampled = true;
            end
        catch
            useSampled = false;
        end
    end

    if ~useSampled
        r = rawData.Robot(robotId);
    
        % --- Groundtruth: sort + unique timestamps ---
        GT = r.Groundtruth;                 % [t x y th]
        GT = sortrows(GT, 1);
        [tGT, iaGT] = unique(GT(:,1), 'stable');
        gt = GT(iaGT, 2:4);                 % [x y th]
    
        % --- Odometry: sort + unique timestamps ---
        O = r.Odometry;                     % [t v w]
        O = sortrows(O, 1);
        [tO, iaO] = unique(O(:,1), 'stable');
        vwO = O(iaO, 2:3);                  % [v w]
    
        % --- Align odometry to GT timeline ---
        vw = interp1(tO, vwO, tGT, 'previous', 'extrap');
    
        % --- Measurements: group by nearest GT time ---
        obs = cell(numel(tGT),1);
        meas = r.Measurement;              % [t barcode r b]
        meas = sortrows(meas, 1);
        for i = 1:size(meas,1)
            tm = meas(i,1);
            [~, k] = min(abs(tGT - tm));
            obs{k} = [obs{k}; meas(i,2:4)];
        end
    
        gui.data.t    = tGT;
        gui.data.gt   = gt;
        gui.data.odom = vw;
        gui.data.obs  = obs;
    
        % landmark ground truth (for plotting/eval)
        if isfield(rawData,'Landmark_Groundtruth')
            gui.data.landmarks_gt = rawData.Landmark_Groundtruth(:,2:3);
        else
            gui.data.landmarks_gt = [];
        end
    end


    % ---- Reset state ----
    gui.k = 1;

    algoName = gui.algoDD.Value;
    gui.est = init_estimator(gui.data, gui.robotId, algoName);
    

    pts = [gui.data.gt_raw(:,1:2); gui.data.landmarks_gt];
    if ~isempty(pts)
        margin = 2.0;
        gui.plotBounds.xlim = [min(pts(:,1))-margin, max(pts(:,1))+margin];
        gui.plotBounds.ylim = [min(pts(:,2))-margin, max(pts(:,2))+margin];
    else
        gui.plotBounds = [];
    end

    cla(gui.ax);
    gui = gui_initPlot(gui);
    gui.estHist = [];
end
