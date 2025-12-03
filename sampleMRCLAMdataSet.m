% Sample MRCLAM dataset using struct "data" created by loadMRCLAMdataSet.m

% Option %
sample_time = 0.02;
% END OPTION %

n_robots = length(data.Robot);

% --- FIND MIN AND MAX TIME ---
min_time = inf;
max_time = -inf;

for i = 1:n_robots
    gt = data.Robot(i).Groundtruth;
    min_time = min(min_time, gt(1,1));
    max_time = max(max_time, gt(end,1));
end

% Shift timestamps
for i = 1:n_robots
    data.Robot(i).Groundtruth(:,1)  = data.Robot(i).Groundtruth(:,1)  - min_time;
    data.Robot(i).Odometry(:,1)     = data.Robot(i).Odometry(:,1)     - min_time;
    data.Robot(i).Measurement(:,1)  = data.Robot(i).Measurement(:,1)  - min_time;
end

max_time = max_time - min_time;

% Compute timesteps
timesteps = floor(max_time/sample_time) + 1;

disp(['time ', num2str(min_time), ' is first timestep (t=0[s])']);
disp(['sampling time = ', num2str(sample_time),' [s] (', num2str(1/sample_time), ' Hz)']);
disp(['resulting timesteps = ', num2str(timesteps)]);

% ---------------------------------------
% RESAMPLE GROUNDTRUTH & ODOMETRY
% ---------------------------------------

for i = 1:n_robots
    fields = {'Groundtruth','Odometry'};
    for f = 1:length(fields)

        old = data.Robot(i).(fields{f});
        [nr,nc] = size(old);

        new = zeros(timesteps,nc);

        k = 0;
        t = 0;
        idx = 1;

        while t <= max_time
            new(k+1,1) = t;

            % find interval
            while old(idx,1) <= t
                if idx == nr
                    break;
                end
                idx = idx + 1;
            end

            % if t outside range -> copy or zero
            if idx == 1 || idx == nr
                new(k+1,2:end) = old(idx,2:end);
            else
                % interpolate linearly
                p = (t - old(idx-1,1)) / (old(idx,1) - old(idx-1,1));
                for c = 2:nc
                    new(k+1,c) = old(idx-1,c) + p * (old(idx,c) - old(idx-1,c));
                end
            end

            k = k + 1;
            t = t + sample_time;
        end

        data.Robot(i).(fields{f}) = new;
    end
end

% ---------------------------------------
% RESAMPLE MEASUREMENTS (round to nearest timestep)
% ---------------------------------------

for i = 1:n_robots
    M = data.Robot(i).Measurement;
    Mnew = M;

    for j = 1:size(M,1)
        Mnew(j,1) = floor(M(j,1)/sample_time + 0.5)*sample_time;
    end

    data.Robot(i).Measurement = Mnew;
end

disp('Sampling complete.');
