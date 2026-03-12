%% ECBF QP control for double-integrator with multiple circular obstacles
clear; clc; close all;

% Simulation params
dt = 0.01;
tf = 20;
time = 0:dt:tf;

% System initial condition
p = [-3; -1]; % position [x,y]
v = [0; 0];   % velocity [vx,vy]
x = [p; v];   % state [x,y,vx,vy]
xx = x;
m = 2; % number of control inputs

% Goal and nominal controller gains
p_star = [2;2];   % target position [xs,ys]
v_star = [0;0];   % target velocity [vxs,vys]
Kp = 1.3; Kd = 2; % PD controller gains

% Obstacles: each row -> {center, radius}
obs = {[-1;0.65], 0.7;
       [1;1], 0.5};


num_obs = size(obs,1);

% ECBF params (h_ddot + a1*h_dot + a2*h >= 0) 
p1 = [10; 5];
p2 = [9;10];
a1 = p1 + p2; % (alpha1 >=0 )
a2 = p1.*p2; % (alpha2 >=0 )

F = [0 1 ; 0 0];
G = [0;1];
K = [a2 a1];

for i = 1:num_obs
    h_dyn = F-G*K(i,:);
    eigenvalues = eig(h_dyn);
    fprintf('Eigenvalues for Obs %d: %s\n', i, num2str(eigenvalues'));
end
  
% QP slack weight (discourage violation)
% gamma = 0;

% Control limits
u_max = 5;
u_min = -5;

% quadprog options
opts = optimoptions('quadprog','Display','off');

% Preallocate logs
Ulog = zeros(2,length(time)-1);
Slog = zeros(num_obs,length(time)-1);
nu2_log = zeros(num_obs,length(time)-1);

for k = 1:length(time)-1
    p = x(1:2); % current position
    v = x(3:4); % current velocity

    % nominal PD control (acceleration command)
    u_nom = Kp*(p_star - p) + Kd*(v_star - v);

    % Build QP matrices
    % Decision vector z = [u(2x1); s(mx1)] control input and slack variable
    % One slack variable for each constraint (obstacle)
    nz = m; % + num_obs;
    H = zeros(nz); % square matrix for quadprog optimisation eqn
    % Put 2*I on u block, 2*gamma*I on slack block so quadprog's 0.5 factor yields desired cost
    H(1:2,1:2) = 2*eye(2);
    % H(3:end,3:end) = 2*gamma*eye(num_obs);

    f = zeros(nz,1);    % linear part of quadprog optimisation eqn
    f(1:2) = -2*u_nom;  % corresponds to -2*u_nom'*u term

    % Inequalities A_qp * z <= b_qp
    A_qp = [];
    b_qp = [];

    % Loop over each constraint (obstacle)
    for i = 1:num_obs
        ci = obs{i,1};
        ri = obs{i,2};

        h_i = (p - ci)'*(p - ci) - ri^2;
        hdot = 2*(p - ci)'*v; 
        % hddot = 2*(v'*v) + 2*(p - ci)'*u; (can't add this yet cause we
        % don't have u but we will use the part without u
        
        % putting in form Au >= b
        A_i = 2*(p - ci)';
        % b_i as derived 
        b_i = - ( 2*(v'*v) + a1(i) * hdot + a2(i) * h_i );

        % store nu2 for diagnostics (nu2 = 2*(p-c)'*u + const_term)
        % We'll compute nu2 later after obtaining u; store const part now
        const_term = 2*(v'*v) + a1(i)*hdot + a2(i)*h_i;
        % building QP constraints for quadprog from our CBF constraints
        % convert a'u - s >= b into: -A_i * u + s_i <= -b_i 
        Aq_row = [-A_i]; %, zeros(1,num_obs)];
        % Aq_row(2 + i) = 0;      % place 1 for s_i (indexing: 1..2 for u, 3..2+m for s)
        A_qp = [A_qp; Aq_row];
        b_qp = [b_qp; -b_i];
    end

    % bounds: enforce s >= 0 and u limits
    lb = [u_min*ones(2,1)];%; zeros(num_obs,1)];
    ub = [u_max*ones(2,1)];%; inf(num_obs,1)];
    % lb = [];
    % ub = [];

    % Solve QP
    z = quadprog(H,f,A_qp,b_qp,[],[],lb,ub,[],opts);
    if isempty(z)
        warning('quadprog failed at step %d — using u_nom (no safety filter)', k);
        u = u_nom;
        % s = inf(num_obs,1);
    else
        u = z(1:2);
        % s = z(3:end);
    end

    % compute nu2 values for logging:
    for i = 1:num_obs
        ci = obs{i,1};
        ri = obs{i,2};
        h_i = (p - ci)'*(p - ci) - ri^2;
        hdot = 2*(p - ci)'*v;
        hddot_no_u = 2*(v'*v);
        nu2 = (2*(p-ci)'*u) + (2*(v'*v) + a1(i)*hdot + a2(i)*h_i);
        nu2_log(i,k) = nu2;
    end

    % Integrate (simple forward Euler)
    p_new = p + v*dt;
    v_new = v + u*dt;
    x = [p_new; v_new];

    % Logging
    xx = [xx x];
    Ulog(:,k) = u;
    % Slog(:,k) = s;
end

%% Plot results
figure;
subplot(2,2,1); hold on; axis equal; grid on;
plot(xx(1,:), xx(2,:), 'b-', 'LineWidth',1.5);
plot(p_star(1), p_star(2), 'rx','MarkerSize',10,'LineWidth',2);
theta = linspace(0,2*pi,120);
for i = 1:num_obs
    ci = obs{i,1}; ri = obs{i,2};
    plot(ci(1)+ri*cos(theta), ci(2)+ri*sin(theta), 'r-','LineWidth',1.2);
end
title('Trajectory'); xlabel('x'); ylabel('y');

subplot(2,2,2);
plot(time(1:end-1), Ulog(1,:), time(1:end-1), Ulog(2,:));
legend('u_x','u_y'); xlabel('t'); ylabel('u');

% subplot(2,2,3);
% plot(time(1:end-1), Slog');
% xlabel('t'); ylabel('slack s_i'); title('Slack variables');

subplot(2,2,4); hold on;
for i = 1:num_obs
    plot(time(1:end-1), nu2_log(i,:));
end
yline(0,'k--');
legend(arrayfun(@(i)sprintf('nu2 obs %d',i),1:num_obs,'UniformOutput',false));
xlabel('t'); ylabel('\nu_2'); title('\nu_2 values (should stay >= 0)');
