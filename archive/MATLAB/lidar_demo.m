clear
clc
clf

%% PARAMETERS

buffer = 0.0;

%% SCAN SIMULATION
lidar_data = load('lidar_2026-03-13_19-32-30.txt');

dang = diff(lidar_data(:,1));
edges = find(dang<0 & abs(dang)>3);
edges = [0; edges; size(lidar_data,1)-1];

x_pos = 0.5;
y_pos = 0.5;

fig = figure(1);
ax = axes('Parent', fig);
hold on; axis square; grid on;
xlim([-2,2])
ylim([-2,2])
axis equal

for i=42:75 % 37 length(edges)-1
    disp(i)
    start_i = edges(i)+1;
    end_i = edges(i+1);

    single_scan = lidar_data(start_i:end_i,:);

    angles = single_scan(:,1); % +34
    ranges = 0.001*single_scan(:,2); %+34

    ranges = ranges(angles > 45 & angles < 165);
    angles = -(angles(angles > 45 & angles < 165) + 155);
  
    % ranges = ranges + 0.01*randn(length(ranges),1);

    cla(ax)

    plot(ax, x_pos, y_pos, 'rx','MarkerSize',10,'LineWidth',2);
    [x_test2, y_test2] = pol2cart(deg2rad(angles),ranges);
    plot(x_pos+x_test2, y_pos+y_test2, 'go')

    obs = ProcessLidar(angles,ranges, x_pos, y_pos, buffer);

    for i=1:size(obs,1)
        if ~isempty(obs{1})
            viscircles([obs{i,1}(1), obs{i,1}(2)], obs{i,2});
        end
    end

    drawnow;
    pause(0.1)

end


function obs = ProcessLidar(angles, ranges, x_pos, y_pos, buffer)

    %% --- Parameters ---
    jump_threshold     = 0.1;   % Min range jump (m) to count as an edge
    min_segment_pts    = 3;      % Ignore segments with fewer points
    max_segment_pts    = 60;     % Ignore suspiciously large segments (background)
    curvature_thresh   = 0.03;   % How non-linear a segment must be (higher = stricter)
    min_radius         = 0.2;   % Min plausible obstacle radius (m)
    max_radius         = 0.5;    % Max plausible obstacle radius (m)
    range_min          = 0.1;    % Ignore returns closer than this (noise)
    range_max          = 1.5;    % Ignore returns further than this

    %% --- Filter bad ranges ---
    valid = ranges > range_min & ranges < range_max;
    angles = angles(valid);
    ranges = ranges(valid);
    [X_all, Y_all] = pol2cart(deg2rad(angles), ranges);
    grad = gradient(Y_all,X_all);
    grad_slope = abs(diff(grad))';

    flat = grad_slope < 0.1;

    d = diff([0 flat 0]);     % detect transitions
    start_idx = find(d == 1);
    end_idx   = find(d == -1) - 1;

    lengths = end_idx - start_idx + 1;

    [~, k] = max(lengths);

    longest_start = start_idx(k);
    longest_end   = end_idx(k);

    plot(x_pos+X_all(longest_start), y_pos+Y_all(longest_start), 'rx')
    plot(x_pos+X_all(longest_end), y_pos+Y_all(longest_end), 'rx')

    coefficients = polyfit(X_all(longest_start:longest_end)+x_pos, Y_all(longest_start:longest_end)+y_pos, 1);
    yFit = polyval(coefficients, X_all+x_pos);
    plot(X_all+x_pos, yFit, 'r-', 'LineWidth', 2); % Plot fit line

    % [X_all, Y_all] = pol2cart(deg2rad(angles), ranges);
    % n_pts = length(X_all);
    % 
    % %% --- Robust Straight-Line Detection ---
    % window_size = 12;  % Look at ~12 points at a time
    % r_sq = zeros(1, n_pts);
    % 
    % for i = 1:(n_pts - window_size)
    %     idx = i : i + window_size;
    %     % Calculate correlation between X and Y
    %     % r(1,2) is the correlation coefficient
    %     r = corrcoef(X_all(idx), Y_all(idx));
    %     r_sq(i + floor(window_size/2)) = abs(r(1,2)); 
    % end
    % 
    % % Points are "straight" if they have a very high linear correlation
    % % 0.99 is very strict; 0.97 is more robust to heavy noise
    % is_straight = r_sq > 0.95;
    % 
    % % Group these into the longest straight segment
    % d = diff([0, is_straight, 0]);
    % start_idx = find(d == 1);
    % end_idx   = find(d == -1) - 1;
    % 
    % if isempty(start_idx)
    %     obs = cell(0,2); return; 
    % end
    % 
    % [~, k] = max(end_idx - start_idx + 1);
    % longest_start = start_idx(k);
    % longest_end   = end_idx(k);
    % 
    % %% --- Robust Line Fitting (Total Least Squares) ---
    % % Instead of polyfit (which assumes Y depends on X), 
    % % we use a simple PCA-based line fit for the "longest" segment.
    % segX = X_all(longest_start:longest_end);
    % segY = Y_all(longest_start:longest_end);
    % 
    % mx = mean(segX);
    % my = mean(segY);
    % 
    % % pts_centered is [N x 2]
    % pts_centered = [segX - mx, segY - my];
    % 
    % % 2. Run SVD
    % % V is the rotation matrix. The first column of V is the 
    % % principal direction (the direction the line is pointing).
    % [~, ~, V] = svd(pts_centered, 0);
    % dirVec = V(:,1); % This is your [dx; dy] unit vector
    % 
    % % 3. Project ALL points onto this line to get the fitted baseline
    % % We center all points, project them onto the direction vector, 
    % % and then shift them back to the original position.
    % all_pts_centered = [X_all - mx, Y_all - my];
    % t = all_pts_centered * dirVec; % Distance along the line for each point
    % 
    % xFit = mx + t * dirVec(1) + x_pos;
    % yFit = my + t * dirVec(2) + y_pos;
    % 
    % plot(xFit, yFit, 'r-', 'LineWidth', 2);

    %% --- Detect segment boundaries via range jumps ---
    % Only flag jumps LARGER than a hard threshold (not statistics-based)
    residuals = abs(yFit - (Y_all + y_pos));
    res_delta = abs(diff(residuals));
    sensitivity_threshold = 0.02; 
    jump_idx = find(residuals > jump_threshold | [0; res_delta] > sensitivity_threshold)';
    breaks = find(diff(jump_idx) > 1);
    starts = [1, breaks + 1];
    ends = [breaks, length(jump_idx)];
    X_groups = cell(1, length(starts));
    Y_groups = cell(1, length(starts));
    for i = 1:length(starts)
        % Get the actual index range for this clump
        % This grabs jump_idx(1) to jump_idx(4), then jump_idx(5) to jump_idx(end)
        actual_indices = jump_idx(starts(i):ends(i));
        
        % Slice your coordinate data using these indices
        X_groups{i} = X_all(actual_indices);
        Y_groups{i} = Y_all(actual_indices);
    end
    % Collect all significant indices
    obs = cell(1, 2);
    num_obs=0;

    %% --- Process each segment ---
    for k = 1:length(X_groups)
        X = X_groups{k};
        Y = Y_groups{k};
        n       = length(X);
        
        if n < min_segment_pts || n > max_segment_pts
            continue;
        end

        %% --- Fit a circle to the segment ---
        A      = [2*X(:), 2*Y(:), ones(n,1)];
        b      = X(:).^2 + Y(:).^2;
        params = A \ b;
        xc     = params(1);
        yc     = params(2);
        R      = sqrt(max(params(3) + xc^2 + yc^2, 0));

        num_obs = num_obs + 1;
        obs{num_obs, 1} = [x_pos + xc, y_pos + yc];
        obs{num_obs, 2} = R + buffer;
    end

    obs = obs(1:num_obs, :);
end