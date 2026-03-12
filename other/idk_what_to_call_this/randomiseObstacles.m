function obstacles = randomiseObstacles(num_obs, xlim_box, ylim_box, exclusion_zones,obstacles)
% RANDOMIZEOBSTACLESRECT Generate random obstacles avoiding rectangular zones.
%
%   obstacles = randomizeObstaclesRect(num_obs, xlim_box, ylim_box, exclusion_zones)
%
%   xlim_box = [xmin xmax]
%   ylim_box = [ymin ymax]
%   exclusion_zones = struct array with fields:
%       - xlim: [xmin xmax]
%       - ylim: [ymin ymax]
%
%   Output:
%       obstacles: cell array of { [x; y], radius }

    rng('shuffle'); % randomize seed

    % Half small radius, half large
    small_r = 0.3;
    large_r = 0.5;

    count = 0;
    while count < num_obs
        % Random candidate position
        x = rand() * (xlim_box(2) - xlim_box(1)) + xlim_box(1);
        y = rand() * (ylim_box(2) - ylim_box(1)) + ylim_box(1);

        % Check if inside any rectangular exclusion zone
        valid = true;
        for j = 1:numel(exclusion_zones)
            if x >= exclusion_zones(j).xlim(1) && x <= exclusion_zones(j).xlim(2) && ...
               y >= exclusion_zones(j).ylim(1) && y <= exclusion_zones(j).ylim(2)
                valid = false;
                break;
            end
        end

        % If not excluded, accept the obstacle
        if valid
            count = count + 1;
            radius = (rand() < 0.5) * (large_r - small_r) + small_r; % random small or large
            obstacles{count, 1} = [x; y];
            obstacles{count, 2} = radius;
        end
    end
end
