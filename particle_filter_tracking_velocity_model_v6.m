% ===============================
% Initial settings
% ================================
clear
close all
rng(200); % Set seed for random number generation

% Load video file
[file, path] = uigetfile('*.avi','Select the video file');

videoFile = [path '\' file]; % Video file name
videoReader = VideoReader(videoFile); % Read video

% Initial parameters
numFrames = videoReader.NumFrames; % Number of frames
frameRate = videoReader.FrameRate; % Frame rate
width = videoReader.Width; % Video width
height = videoReader.Height; % Video height

show_particles = true; % Particle display mode

N_PARTICLES = 200; % Number of particles per object
euclidean_dist_thresh = 100; % Distance threshold for assignment
max_track_strikes = 10; % Maximum number of frames without detection before object removal
initial_estimate_covariance = [60, 2, 60,2]; % Initial covariance
initial_x_dot = 0.5; % Initial x-velocity
initial_y_dot = 0.5; % Initial y-velocity
process_noise = 1*[10, 2, 10,2]; % Process noise
sensor_noise = 2; % Sensor noise
dt = 1/30; % Time interval

% ==============================
% Get settings from user
% ===============================
color = rand(3000,3)*255; % Random color for each object (you can also define fixed colors)

output_folder = uigetdir('', 'Select output folder to save images');
if output_folder == 0
    error('Operation canceled. Application stopped.');
end

trajectory_length =10;% input('Enter the length of the displayed trajectory (number of frames): ');
if isempty(trajectory_length)
    trajectory_length = 50;
end

% ===============================
% Initialize the tracking variables (important)
% ===============================
all_particles = {}; % Each element: N_PARTICLESx4 [x, x_dot, y, y_dot]
all_weights = {};
all_track_ids = [];
all_track_strikes = [];
all_track_classes = {};
id_count = 0;
all_last_positions={};
trajectories = containers.Map('KeyType','int32','ValueType','any'); % Trajectories

% ===============================
% Prepare CSV file
% =================================
csv_path = fullfile(output_folder, 'Particle_filter_veloc_trajectories.csv');
fid_csv = fopen(csv_path, 'w');
fprintf(fid_csv, 'Frame,Track_ID,Class,X,Y\n');

% ================================
% Image processing loop
% ================================
for frame_idx = 1:numFrames

    % Read frame
    frame = read(videoReader, frame_idx);

    frame_gray = rgb2gray(frame);
    frame_binary = imbinarize(frame_gray);
    s = regionprops(frame_binary,'centroid');
    current_detections = cat(1,s.Centroid);

    % Display frame number on image
    frame = insertText(frame, [10 10], sprintf('Frame: %d', frame_idx), 'FontSize', 18, 'BoxColor', 'yellow', 'BoxOpacity', 0.6);

    % Draw detection boxes (circles on points)
    for d = 1:size(current_detections,1)
        frame = insertShape(frame, 'Circle', [current_detections(d,1), current_detections(d,2), 10], 'Color', 'red', 'LineWidth', 2);
    end

    % Prediction for all particles
    for i = 1:length(all_particles)
        particles = all_particles{i};
        if all_track_strikes(i) ==0
            % Particle motion model (constant velocity)
            particles(:,1) = particles(:,1) + particles(:,2)*dt +randn(N_PARTICLES,1)*process_noise(1);
            particles(:,2) = particles(:,2) + randn(N_PARTICLES,1)*process_noise(2);
            particles(:,3) = particles(:,3) + particles(:,4)*dt+randn(N_PARTICLES,1)*process_noise(3);
            particles(:,4) = particles(:,4) +randn(N_PARTICLES,1)*process_noise(4);
        else
            dt_1 = 1/30;
            particles(:,1) = particles(:,1) + particles(:,2)*dt_1 +randn(N_PARTICLES,1)*process_noise(1);
            particles(:,2) = particles(:,2) + randn(N_PARTICLES,1)*process_noise(2);
            particles(:,3) = particles(:,3) + particles(:,4)*dt_1+randn(N_PARTICLES,1)*process_noise(3);
            particles(:,4) = particles(:,4) +randn(N_PARTICLES,1)*process_noise(4);
        end

        all_particles{i} = particles;
    end

    % If detection exists
    if ~isempty(current_detections)
        if isempty(all_particles)
            % Create particles for each new detection
            for d = 1:size(current_detections,1)
                det_center_x = current_detections(d,1);
                det_center_y = current_detections(d,2);
                particles = zeros(N_PARTICLES,6);
                particles(:,1) = normrnd(det_center_x, initial_estimate_covariance(1), N_PARTICLES,1);
                particles(:,2) = normrnd(initial_x_dot, initial_estimate_covariance(2), N_PARTICLES,1);

                particles(:,3) = normrnd(det_center_y, initial_estimate_covariance(3), N_PARTICLES,1);
                particles(:,4) = normrnd(initial_y_dot, initial_estimate_covariance(4), N_PARTICLES,1);

                weights = ones(N_PARTICLES,1) / N_PARTICLES;
                all_particles{end+1} = particles;
                all_weights{end+1} = weights;
                all_track_ids(end+1) = id_count;
                all_track_strikes(end+1) = 0;
                all_track_classes{end+1} = 'target'; % Hypothetical class
                id_count = id_count + 1;
                all_last_positions{end+1} = current_detections(d,:);
            end
        else
            % --------- Improved Data Association and Track Management ---------
            % Calculate the average particle position for each object
            all_track_mean_position = zeros(length(all_particles),2);
            for i=1:length(all_particles)
                particles = all_particles{i};
                weights = all_weights{i};
                mean_pos = sum(particles(:,[1,3]) .* weights, 1);

                mean_veloc = sum(particles(:,[2,4]) .* weights, 1);

                %                 all_track_mean_position(i,:) = mean_pos+mean_veloc*dt;
                %                 all_track_mean_position(i,:) = mean_pos;
                if all_track_strikes(i)==0
                    %                             all_track_mean_position(i,:) = mean_pos+(mean_pos-all_last_positions{i})*dt;
                    all_track_mean_position(i,:) = mean_pos+mean_veloc*dt;

                else
                    dt_1 = 1/30;
                    %                             all_track_mean_position(i,:) = mean_pos+(mean_pos-all_last_positions{i})*dt_1;
                    all_track_mean_position(i,:) = mean_pos+mean_veloc*dt_1;

                end

                all_last_positions{i} = mean_pos;


            end

            detection_center_positions = current_detections;

            % Hungarian/Munkres assignment
            if ~isempty(all_track_mean_position) && ~isempty(detection_center_positions)
                %     The cost matrix now includes a large penalty for assignments where the detection is not
                % in the predicted direction of the track's velocity.
                %
                %     This helps the Munkres algorithm maintain label consistency after targets cross or come close.
                direction_angle_cosine_thresh = 0.05;
                direction_penalty = euclidean_dist_thresh/4;

                cost_matrix = zeros(size(all_track_mean_position,1), size(detection_center_positions,1));
                for i = 1:size(all_track_mean_position,1)
                    mean_pos = all_track_mean_position(i,:)';
                    particles = all_particles{i};
                    weights = all_weights{i};
                    mean_veloc = sum(particles(:,[2,4]) .* weights, 1);
                    for j = 1:size(detection_center_positions,1)
                        det_pos = detection_center_positions(j,:)';
                        eucl_dist = norm(mean_pos - det_pos);
                        to_det_vec = det_pos - mean_pos;
                        if norm(mean_veloc) > 1e-3 && norm(to_det_vec) > 1e-3
                            cos_theta = dot(mean_veloc, to_det_vec) / (norm(mean_veloc)*norm(to_det_vec));
                        else
                            cos_theta = 1;
                        end
                        if cos_theta < direction_angle_cosine_thresh
                            %                         cost_matrix(i,j) = eucl_dist + direction_penalty;
                            cost_matrix(i,j) = eucl_dist + direction_penalty*(1-cos_theta);

                        else
                            cost_matrix(i,j) = eucl_dist;
                        end
                    end
                end


                % manual implementation of Munkres algorithm----
                [row_ind, col_ind] = munkres(cost_matrix,euclidean_dist_thresh);
                assignments = [row_ind col_ind];
                assigned_tracks = assignments(:,1);
                unassigned_tracks = setdiff(1:size(all_track_mean_position,1), row_ind);
                unassigned_detections = setdiff(1:size(detection_center_positions,1), col_ind);
                %-------------
                % Update assigned tracks
                for i = assigned_tracks'
                    [r,~,~] = find(assignments(:,1)==i);
                    det_idx = assignments(r,2);
                    track_idx = i;
                    if det_idx > 0 && cost_matrix(track_idx,det_idx) <= euclidean_dist_thresh
                        all_track_strikes(track_idx) = 0;
                        particles = all_particles{track_idx};
                        weights = all_weights{track_idx};
                        mean_pos = sum(particles(:,[1,3]) .* weights, 1);
                        dist_from_det_to_mean = norm(mean_pos - detection_center_positions(det_idx,:));
                        dist_particles = sqrt(sum((particles(:,[1,3]) - detection_center_positions(det_idx,:)).^2, 2));
                        weights = normpdf(dist_particles, dist_from_det_to_mean, sensor_noise)+1e-300;
                        weights = max(weights,eps);
                        weights = weights / sum(weights);
                        all_weights{track_idx} = weights;
                        % Resample if needed
                        neff = 1 / sum(weights.^2);
                        if neff < N_PARTICLES/2
                            indexes = systematic_resample(weights);
                            particles = particles(indexes,:);
                            weights = ones(N_PARTICLES,1)/N_PARTICLES;
                            all_particles{track_idx} = particles;
                            all_weights{track_idx} = weights;
                        end
                    else
                        all_track_strikes(track_idx) = all_track_strikes(track_idx) + 1;
                    end
                end

                % Try to re-associate unassigned tracks with close detections (track recovery)
                if ~isempty(unassigned_tracks)

                    for i = unassigned_tracks
                        % Only attempt re-association if strikes < max_track_strikes
                        if all_track_strikes(i) < max_track_strikes
                            % Find the nearest unassigned detection
                            dists = cost_matrix(i, unassigned_detections);
                            [min_dist, min_idx] = min(dists);
                            if ~isempty(min_dist) && min_dist <= euclidean_dist_thresh
                                det_idx = unassigned_detections(min_idx);
                                % Re-associate this track with the detection
                                all_track_strikes(i) = 0;
                                particles = all_particles{i};
                                weights = all_weights{i};
                                mean_pos = sum(particles(:,[1,3]) .* weights, 1);
                                dist_from_det_to_mean = norm(mean_pos - detection_center_positions(det_idx,:));
                                dist_particles = sqrt(sum((particles(:,[1,3]) - detection_center_positions(det_idx,:)).^2, 2));
                                weights = normpdf(dist_particles, dist_from_det_to_mean, sensor_noise);
                                weights = max(weights,eps);
                                weights = weights / sum(weights);
                                all_weights{i} = weights;
                                % Resample if needed
                                neff = 1 / sum(weights.^2);
                                if neff < N_PARTICLES/2
                                    indexes = systematic_resample(weights);
                                    particles = particles(indexes,:);
                                    weights = ones(N_PARTICLES,1)/N_PARTICLES;
                                    all_particles{i} = particles;
                                    all_weights{i} = weights;
                                end
                                % Remove this detection from unassigned_detections
                                unassigned_detections(min_idx) = [];
                            else
                                % No close detection found, increase strike
                                all_track_strikes(i) = all_track_strikes(i) + 1;
                            end
                        else
                            all_track_strikes(i) = all_track_strikes(i) + 1;
                        end
                    end
                end

                % Remove tracks with too many strikes
                i = 1;
                while i <= length(all_particles)
                    if all_track_strikes(i) >= max_track_strikes
                        fprintf('Track %d removed!\n', all_track_ids(i));
                        all_particles(i) = [];
                        all_weights(i) = [];
                        all_track_ids(i) = [];
                        all_track_strikes(i) = [];
                        all_track_classes(i) = [];
                    else
                        i = i + 1;
                    end
                end

                % Create new tracks for remaining unassigned detections
                if ~isempty(unassigned_detections)
                    for idx = unassigned_detections
                        det_center_x = detection_center_positions(idx,1);
                        det_center_y = detection_center_positions(idx,2);
                        particles = zeros(N_PARTICLES,4);
                        particles(:,1) = normrnd(det_center_x, initial_estimate_covariance(1), N_PARTICLES,1);
                        particles(:,2) = normrnd(initial_x_dot, initial_estimate_covariance(2), N_PARTICLES,1);

                        particles(:,3) = normrnd(det_center_y, initial_estimate_covariance(3), N_PARTICLES,1);
                        particles(:,4) = normrnd(initial_y_dot, initial_estimate_covariance(4), N_PARTICLES,1);

                        weights = ones(N_PARTICLES,1) / N_PARTICLES;
                        all_particles{end+1} = particles;
                        all_weights{end+1} = weights;
                        all_track_ids(end+1) = id_count;
                        all_track_strikes(end+1) = 0;
                        all_track_classes{end+1} = 'target';
                        id_count = id_count + 1;
                        all_last_positions{end+1} = detection_center_positions(idx,:);

                    end
                end
            end
            % --------- End Improved Data Association and Track Management ---------
        end
    end

    % Update routes and save to CSV
    current_positions = [];
    for i = 1:length(all_particles)
        particles = all_particles{i};
        weights = all_weights{i};
        track_id = all_track_ids(i);
        track_class = all_track_classes{i};
        mean_pos = sum(particles(:,[1,3]) .* weights, 1);
        x = round(mean_pos(1));
        y = round(mean_pos(2));

        if ~isKey(trajectories, track_id)
            trajectories(track_id) = zeros(trajectory_length, 2);
        end
        traj = trajectories(track_id);
        traj = [traj(2:end,:); [x, y]];
        trajectories(track_id) = traj;

        % Write to CSV
        fprintf(fid_csv, '%d,%d,%s,%d,%d\n', frame_idx, track_id, track_class, x, y);
    end

    % Draw particles and paths
    for i = 1:length(all_particles)
        particles = all_particles{i};
        weights = all_weights{i};
        track_id = all_track_ids(i);
        track_class = all_track_classes{i};

        if show_particles
            frame = insertShape(frame, 'FilledCircle', [particles(:,1), particles(:,3), 2*ones(N_PARTICLES,1)], 'Color', 'cyan', 'Opacity', 0.5);
        end

        if isKey(trajectories, track_id)
            traj = trajectories(track_id);
            for k = 2:size(traj,1)
                if sum(traj(k-1,:))
                    frame = insertShape(frame, 'Line', [traj(k-1,:), traj(k,:)], 'Color', color(track_id+1,:), 'LineWidth', 2);
                end
            end

        end

        mean_pos = sum(particles(:,[1,3]) .* weights, 1);
        frame = insertShape(frame, 'FilledCircle', [mean_pos(1), mean_pos(2), 5], 'Color', 'white');
        frame = insertText(frame, [mean_pos(1), mean_pos(2)-20], sprintf('%s %d', track_class, track_id), 'FontSize', 14, 'BoxColor', color(track_id+1,:)/255, 'TextColor', 'green');
    end

    % Save output image
    imwrite(frame, [output_folder '\' num2str(frame_idx) '.jpg']);

    % Show frame
    figure(1)
    imshow(frame);
    title(sprintf('Frame %d', frame_idx));
    drawnow;
    pause(0.001)
    % Press q to exit (requires active Figure window)
    key = get(gcf,'CurrentCharacter');
    if key == 'q'
        break;
    end
end

fclose(fid_csv);

% ============================
% Helper functions
% ================================
% (add munkres and systematic_resample functions here or make sure they are in your MATLAB path)