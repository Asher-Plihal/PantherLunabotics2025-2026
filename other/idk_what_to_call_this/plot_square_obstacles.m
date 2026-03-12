function plot_square_obstacles(obstacles,ax)
%PLOT_SQUARE_OBSTACLES  Plot superlevel sets for multiple square obstacles
%
%   PLOT_SQUARE_OBSTACLES(OBSTACLES) takes a cell array of obstacle
%   structures and plots their superlevel set boundaries.
%
%   Each obstacle struct must include:
%       bx, by : center coordinates
%       ax, ay : half-lengths or scaling parameters
%       cj     : constant offset
%       p      : shape parameter (controls squareness)
%
%   Example:
%       obstacles = {
%           struct('bx', 2, 'by', 2, 'ax', 1, 'ay', 1, 'cj', 1, 'p', 20), ...
%           struct('bx', -3, 'by', 1, 'ax', 1.5, 'ay', 1.5, 'cj', 1, 'p', 20), ...
%           struct('bx', 0, 'by', -3, 'ax', 2, 'ay', 2, 'cj', 1, 'p', 20)
%       };
%       plot_square_obstacles(obstacles);

    if nargin < 1 || isempty(obstacles)
        % Default obstacles if none provided
        obstacles = {
            struct('bx', 2, 'by', 2, 'ax', 1, 'ay', 1, 'cj', 1, 'p', 20), ...
            struct('bx', -3, 'by', 1, 'ax', 1.5, 'ay', 1.5, 'cj', 1, 'p', 20), ...
            struct('bx', 0, 'by', -3, 'ax', 2, 'ay', 2, 'cj', 1, 'p', 20)
        };
    end

    % Grid for evaluation
    [xq, yq] = meshgrid(-6:0.05:6, -6:0.05:6);

    hold on;
    colormap('jet');

    % Loop through obstacles
    for j = 1:length(obstacles)
        obs = obstacles{j};

        % Compute h_j(x) on the grid
        hx = ( ((xq - obs.bx) * obs.ax).^obs.p + ((yq - obs.by) * obs.ay).^obs.p ).^(1/obs.p) - obs.cj;

        % Plot zero-superlevel set (boundary)
        contour(ax, xq, yq, hx, [0 0], 'LineWidth', 2);
    end

    xlabel('q_x'); ylabel('q_y');
    title('Square Obstacles as Superlevel Sets');
    grid on;
end