%% ECBF QP control for double-integrator with multiple circular obstacles
clear; clc; close all;

% Simulation params
dt = 0.05;
tf = 15*60;
time = 0:dt:tf;

% System initial condition
x = [1.0; 1.5; pi/2; 0.0];   % state [x,y,theta,v]
xx = x;
m = 2; % number of control inputs [a, w]

% Goal and nominal controller gains
goal_states = {[5.38; 0.6; 0; 0]
                [1.5; 8.0; 0; 0]};   % target position 
Kp = 3; Kd = 0.1; % Linear PD controller gains
Kp_theta = 3.0; Kd_theta = 0.6; % Angular PD controller gains
Kp_pos_ff = 0.2;

% Obstacles: 
exclusion_zones = [
    struct('xlim', [0.0, 2.0], 'ylim', [0.0, 2.0]);  % bottom-left
    struct('xlim', [3.5, 6.88], 'ylim', [0.0, 1.5]);   % middle block
];
xlim_box = [0, 6.88];
ylim_box = [0, 11];
all_obs = randomiseObstacles(10,xlim_box,ylim_box,exclusion_zones);
% center_pole = {struct('bx', 3.4, 'by', 4.5, 'ax', 0.3, 'ay', 0.3, 'cj', 1, 'p', 20)};
all_obs(end+1,:) = {[3.4; 4.5], 0.6};
obs = {[3.4; 4.5], 0.6};

rob_diam = 0.3;

num_obs = 0;
num_cbf = 2;

% ECBF params (h_ddot + a1*h_dot + a2*h >= 0) 
p1 = 1.0;
p2 = 1.0;
a1 = p1 + p2; % (alpha1 >=0 )
a2 = p1.*p2; % (alpha2 >=0 )

F = [0 1 ; 0 0];
G = [0;1];
K = [a2 a1];

% for i = 1:num_obs
h_dyn = F-G*K;
eigenvalues = eig(h_dyn);
fprintf('Eigenvalues for Obs: %s\n', num2str(eigenvalues'));
% end

% QP slack weight (higher values discourage violation)
gamma = 1e-4;
k_qp = 0.01;

% Control limits
u_max = 4;
u_min = -4;
v_max = 0.2;
pos_tol = 0.1;

% quadprog options
opts = optimoptions('quadprog','Display','off');

% Preallocate logs
Ulog = [];
Slog = {};
nu2_log = {};
error = [];

t = 0;
j = 1;
while t < tf
    xs = goal_states{2^(mod(j,2))};
    % Initialize previous errors
    e_pos = sqrt((xs(1) - x(1))^2 + (xs(2) - x(2))^2);
    theta_des = atan2((xs(2) - x(2))^2, (xs(1) - x(1))^2);
    e_ang = theta_des - x(3);
    prev_e_ang = 0;
    prev_e_vel = 0;
    
    while (e_pos > pos_tol) && t < tf
        obs = checkObstacles(x,obs,all_obs);
        num_obs = size(obs,1);
        dx = xs(1) - x(1);
        dy = xs(2) - x(2);
        e_pos = sqrt(dx^2 + dy^2); % distance error
        theta_des = atan2(dy, dx); % desired heading
        e_ang = theta_des - x(3);      % heading error
        e_ang = atan2(sin(e_ang), cos(e_ang)); % wrap to [-pi,pi]
        v_des = min(v_max, 0.2 * e_pos);
        e_vel = v_des - x(4);
    
        % Derivative of errors
        e_ang_dot = (e_ang - prev_e_ang)/dt;
        e_vel_dot = (e_vel - prev_e_vel)/dt;
    
        % PD control
        a = Kp*e_vel + Kd*e_vel_dot + Kp_pos_ff * e_pos;
        w = Kp_theta*e_ang + Kd_theta*e_ang_dot;
        u_nom = [a; w];

        prev_e_ang = e_ang;
        prev_e_vel = e_vel;
    
        % Build QP matrices
        % Decision vector z = [u(2x1); s(mx1)] control input and slack variable
        % One slack variable for each constraint, 2 cbf per obstacle
        nz = m + num_cbf * num_obs;
        % H and f for 0.5*z'Hz + f'z  -> we want (u-u_nom)' W (u-u_nom) + slack_penalty*sum(s^2)
        H = zeros(nz); % square matrix for quadprog optimisation eqn
        % Put 2*I on u block, 2*gamma*I on slack block so quadprog's 0.5 factor yields desired cost
        H(1,1) = 2 * 2;
        H(2,2) = 2 * 0.01;
        H(3:end,3:end) = 2*gamma*eye(num_cbf * num_obs);
    
        f = zeros(nz,1);    % linear part of quadprog optimisation eqn
        f(1:2) = -2 * [2 * u_nom(1); 0.01 * u_nom(2)];  % corresponds to -2*u_nom'*u term
    
        % Inequalities A_qp * z <= b_qp
        A_qp = [];
        b_qp = [];
    
        % Loop over each constraint (obstacle)
        for i = 1:num_obs
            ci = obs{i,1};
            ri = obs{i,2} + rob_diam/2;
    
            h_d_i = (x(1) - ci(1))^2 + (x(2) - ci(2))^2 - ri^2;
            h_d_dot = 2*x(4)*cos(x(3))*(x(1) - ci(1))' + 2*x(4)*sin(x(3))*(x(2) - ci(2))';     
    
            % putting in form Au >= b
            A_i = [2*cos(x(3))*(x(1) - ci(1)) + 2*sin(x(3))*(x(2) - ci(2)) 2*x(4)*cos(x(3))*(x(2) - ci(2)) - 2*x(4)*sin(x(3))*(x(1) - ci(1))];
            % b_i as derived 
            b_i = - ( 2*(x(4)'*x(4)) + a1 * h_d_dot + a2 * h_d_i );
    
            % building QP constraints for quadprog from our CBF constraints
            % convert a'u - s >= b into: -A_i * u + s_i <= -b_i 
            Aq_row = [-A_i, zeros(1,num_cbf*num_obs)];
            Aq_row(m + num_cbf*(i-1) + 1) = 1;      % place 1 for s_i (indexing: 1..2 for u, 3..2+m for s)
            A_qp = [A_qp; Aq_row];
            b_qp = [b_qp; -b_i];    
        end
       
        % bounds: enforce s >= 0 and u limits
        lb = [u_min*ones(2,1); zeros(num_cbf*num_obs,1)];
        ub = [u_max*ones(2,1); inf(num_cbf*num_obs,1)];
    
        % Solve QP
        z = quadprog(H,f,A_qp,b_qp,[],[],lb,ub,[],opts);
        if isempty(z) 
            warning('quadprog failed at step %d — using u_nom (no safety filter)', k);
            u = u_nom;
            s = inf(num_cbf*num_obs,1);
        else
            u = z(1:2);
            s = z(3:end);
        end
    
        % compute nu2 values for logging:
        for i = 1:num_obs
            ci = obs{i,1};
            ri = obs{i,2};
            h_d_i = (x(1) - ci(1))^2 + (x(2) - ci(2))^2 - ri^2;
            h_d_dot = 2*x(4)*cos(x(3))*(x(1) - ci(1))' + 2*x(4)*sin(x(3))*(x(2) - ci(2))';
            hddot_no_u = 2*(x(4)'*x(4));
            nu2(i) = ([2*cos(x(3))*(x(1) - ci(1)) + 2*sin(x(3))*(x(2) - ci(2)) 2*x(4)*cos(x(3))*(x(2) - ci(2)) - 2*x(4)*sin(x(3))*(x(1) - ci(1))]*u) + (2*(x(4)'*x(4)) + a1*h_d_dot + a2*h_d_i);
        end
        nu2_log{end+1} = nu2;
    
        % Integrate (simple forward Euler)
        x3_new = x(3) + u(2)*dt;
        x4_new = x(4) + u(1)*dt;
        x1_new = x(1) + x4_new*cos(x3_new)*dt;
        x2_new = x(2) + x4_new*sin(x3_new)*dt;
        
        x = [x1_new; x2_new; x3_new; x4_new];
    
        % Logging
        xx = [xx x];
        error = [error [e_pos; e_ang]];
        Ulog = [Ulog u];
        Slog{end+1} = s;
        t = t + dt;
    end
    j = j + 1;
end

%% Plot results
fig = figure;
ax = axes('Parent', fig);
hold on; axis square; grid on;
plot(ax, xx(1,:), xx(2,:), 'b-', 'LineWidth',1.5);
for i = 1: size(goal_states,1)
    goals = goal_states{i};
    plot(ax, goals(1), goals(2), 'rx','MarkerSize',10,'LineWidth',2);
end
% plot(xp, yp, 'r-','LineWidth',1.2);
theta = linspace(0,2*pi,120);
for i = 1:num_obs
    ci = obs{i,1}; ri = obs{i,2}/2;
    plot(ax, ci(1)+ri*cos(theta), ci(2)+ri*sin(theta), 'r-','LineWidth',1.2);
    plot(ax, ci(1)+(ri+rob_diam/2)*cos(theta), ci(2)+(ri+rob_diam/2)*sin(theta), 'g:','LineWidth',1.2);
end
title('Trajectory'); xlabel('x'); ylabel('y');
axis equal
% plot_square_obstacles({struct('bx', 3.44, 'by', 5.5, 'ax', 0.2907, 'ay', 0.1818, 'cj', 1, 'p', 20)},ax)

% ax = sign(Ulog(1,:).*cos(xx(3,2:end)));
% ay = sign(Ulog(1,:).*sin(xx(3,2:end)));

% subplot(2,2,3);
% plot(0:dt:(size(xx,2)-2)*dt, Slog);
% xlabel('t'); ylabel('slack s_i'); title('Slack variables');

% subplot(2,2,4); hold on;
% for i = 1:num_obs
%     plot(0:dt:(size(xx,2)-2)*dt, nu2_log(:,i));
% end
% yline(0,'k--');
% legend(arrayfun(@(i)sprintf('nu2 obs %d',i),1:num_obs,'UniformOutput',false));
% xlabel('t'); ylabel('\nu_2'); title('\nu_2 values (should stay >= 0)');


figure()
subplot(3,1,1);
plot(0:dt:(size(xx,2)-2)*dt, error(1,:));
legend('$error_{pos}$','interpreter','latex'); xlabel('t'); ylabel('errors');

subplot(3,1,2);
plot(0:dt:(size(xx,2)-2)*dt, error(2,:));
legend('$error_{ang}$','interpreter','latex'); xlabel('t'); ylabel('errors')

subplot(3,1,3);
plot(0:dt:(size(xx,2)-2)*dt, Ulog(1,:), 0:dt:(size(xx,2)-2)*dt, Ulog(2,:));
% plot(0:dt:(size(xx,2)-2)*dt, ax, 0:dt:(size(xx,2)-2)*dt, ay,  0:dt:(size(xx,2)-2)*dt, Ulog(2,:));
legend('$u_x$', '$u_y$', '$u_{\theta}$','interpreter','latex'); xlabel('t'); ylabel('u');
