#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

using namespace Eigen;
using namespace std;

struct Obstacle {
    Vector2d center;
    double radius;
};

struct ExclusionZone {
    double xlim[2];
    double ylim[2];
};

struct GoalState {
    Vector4d state; // [x, y, theta, v]
};

class ECBFUnicycleController {
private:
    // ... (all your existing private members)
    double dt;
    double tf;
    Vector4d x;
    int nx;
    int m;
    vector<GoalState> goal_states;
    double Kp, Kd;
    double Kp_theta, Kd_theta;
    double Kp_pos_ff;
    vector<ExclusionZone> exclusion_zones;
    double xlim_box[2];
    double ylim_box[2];
    int num_obs;
    vector<Obstacle> obs;
    double rob_diam;
    int num_cbf;
    int nz;
    double p1, p2, a1, a2;
    double gamma;
    double k_qp;
    double u_max, u_min, v_max;
    double pos_tol;
    vector<Vector2d> Ulog;
    vector<VectorXd> Slog;
    vector<VectorXd> nu2_log;
    vector<Vector2d> error_log;
    vector<Vector4d> state_log;
    vector<Vector4d> goals_log;
    OsqpEigen::Solver solver;
    bool solver_initialized;
    
public:
    ECBFUnicycleController() {
        dt = 0.05;
        tf = 300.0;
        x << 1.0, 1.5, M_PI/2, 0.0;
        nx = 4;
        m = 2;
        
        GoalState g1, g2;
        g1.state << 1.5, 8.0, 0.0, 0.0;
        g2.state << 5.38, 0.6, 0.0, 0.0;
        goal_states.push_back(g1);
        goal_states.push_back(g2);
        
        Kp = 3.0;
        Kd = 0.1;
        Kp_theta = 3.0;
        Kd_theta = 0.6;
        Kp_pos_ff = 0.2;
        
        ExclusionZone ez1, ez2;
        ez1.xlim[0] = 0.0; ez1.xlim[1] = 2.0;
        ez1.ylim[0] = 0.0; ez1.ylim[1] = 2.0;
        ez2.xlim[0] = 3.5; ez2.xlim[1] = 6.88;
        ez2.ylim[0] = 0.0; ez2.ylim[1] = 1.5;
        exclusion_zones.push_back(ez1);
        exclusion_zones.push_back(ez2);
        
        xlim_box[0] = 0.0; xlim_box[1] = 6.88;
        ylim_box[0] = 0.0; ylim_box[1] = 11.0;
        
        num_obs = 11;
        rob_diam = 0.3;
        
        initializeObstacles();
        
        num_cbf = 2;
        nz = m + num_cbf * num_obs;
        p1 = 1.0;
        p2 = 1.0;
        a1 = p1 + p2;
        a2 = p1 * p2;
        gamma = 1e-4;
        k_qp = 0.01;
        
        u_max = 4.0;
        u_min = -4.0;
        v_max = 0.2;
        pos_tol = 0.1;
        
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver_initialized = false;
    }
    
    void initializeObstacles() {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis_x(xlim_box[0], xlim_box[1]);
        uniform_real_distribution<> dis_y(ylim_box[0], ylim_box[1]);
        uniform_real_distribution<> dis_r(0.2, 0.5);
        
        for (int i = 0; i < num_obs - 1; i++) {
            Obstacle o;
            bool valid = false;
            
            while (!valid) {
                o.center(0) = dis_x(gen);
                o.center(1) = dis_y(gen);
                o.radius = dis_r(gen);
                
                valid = true;
                for (const auto& zone : exclusion_zones) {
                    if (o.center(0) >= zone.xlim[0] && o.center(0) <= zone.xlim[1] &&
                        o.center(1) >= zone.ylim[0] && o.center(1) <= zone.ylim[1]) {
                        valid = false;
                        break;
                    }
                }
            }
            obs.push_back(o);
        }
        
        Obstacle center_pole;
        center_pole.center << 3.4, 4.5;
        center_pole.radius = 0.6;
        obs.push_back(center_pole);
    }
    
    Vector2d solveQP(const Vector2d& u_nom) {
        int n = nz;
        
        // Setup Hessian matrix H
        SparseMatrix<double> H(n, n);
        vector<Triplet<double>> H_triplets;
        H_triplets.push_back(Triplet<double>(0, 0, 2.0 * 2.0));
        H_triplets.push_back(Triplet<double>(1, 1, 2.0 * 0.01));
        for (int i = 2; i < n; i++) {
            H_triplets.push_back(Triplet<double>(i, i, 2.0 * gamma));
        }
        H.setFromTriplets(H_triplets.begin(), H_triplets.end());
        
        // Gradient vector f
        VectorXd f(n);
        f.setZero();
        f(0) = -2.0 * 2.0 * u_nom(0);
        f(1) = -2.0 * 0.01 * u_nom(1);
        
        // Constraint matrices
        int num_constraints = num_obs;
        SparseMatrix<double> A(num_constraints, n);
        vector<Triplet<double>> A_triplets;
        VectorXd b(num_constraints);
        
        for (int i = 0; i < num_obs; i++) {
            Vector2d ci = obs[i].center;
            double ri = obs[i].radius + rob_diam / 2.0;
            
            double h_d_i = pow(x(0) - ci(0), 2) + pow(x(1) - ci(1), 2) - ri * ri;
            double h_d_dot = 2*x(3)*cos(x(2))*(x(0) - ci(0)) + 2*x(3)*sin(x(2))*(x(1) - ci(1));
            
            double A_i0 = 2*cos(x(2))*(x(0) - ci(0)) + 2*sin(x(2))*(x(1) - ci(1));
            double A_i1 = 2*x(3)*cos(x(2))*(x(1) - ci(1)) - 2*x(3)*sin(x(2))*(x(0) - ci(0));
            
            double b_i = -(2*x(3)*x(3) + a1*h_d_dot + a2*h_d_i);
            
            A_triplets.push_back(Triplet<double>(i, 0, -A_i0));
            A_triplets.push_back(Triplet<double>(i, 1, -A_i1));
            A_triplets.push_back(Triplet<double>(i, m + num_cbf*i, 1.0));
            
            b(i) = -b_i;
        }
        A.setFromTriplets(A_triplets.begin(), A_triplets.end());
        
        // Variable bounds
        VectorXd lb(n), ub(n);
        lb(0) = u_min; lb(1) = u_min;
        ub(0) = u_max; ub(1) = u_max;
        for (int i = 2; i < n; i++) {
            lb(i) = 0.0;
            ub(i) = 1e10;
        }
        
        // Constraint bounds
        VectorXd lower_bound = -1e10 * VectorXd::Ones(num_constraints);
        
        // Initialize solver on first call
        if (!solver_initialized) {
            solver.data()->setNumberOfVariables(n);
            solver.data()->setNumberOfConstraints(num_constraints);
            solver.data()->setHessianMatrix(H);
            solver.data()->setGradient(f);
            solver.data()->setLinearConstraintsMatrix(A);
            solver.data()->setLowerBound(lb);
            solver.data()->setUpperBound(ub);
            solver.data()->setBounds(lower_bound, b);
            
            if (!solver.initSolver()) {
                cerr << "QP solver initialization failed!" << endl;
                return u_nom;
            }
            solver_initialized = true;
        } else {
            // Update solver with new data
            solver.updateHessianMatrix(H);
            solver.updateGradient(f);
            solver.updateLinearConstraintsMatrix(A);
            solver.updateBounds(lower_bound, b);
            solver.updateLowerBound(lb);
            solver.updateUpperBound(ub);
        }
        
        // Solve QP
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            cerr << "QP solve failed, using nominal control" << endl;
            return u_nom;
        }
        
        VectorXd z = solver.getSolution();
        Vector2d u = z.head(2);
        
        return u;
    }
    
    void run() {
        double t = 0.0;
        int goal_idx = 0;
        
        while (t < tf) {
            Vector4d xs = goal_states[goal_idx].state;
            
            if (goal_idx == 0) {
                uniform_real_distribution<> dis(-0.5, 0.5);
                random_device rd;
                mt19937 gen(rd());
                xs(0) += dis(gen);
                xs(1) += 2.0 * dis(gen);
            }
            
            goals_log.push_back(xs);
            
            double e_pos = sqrt(pow(xs(0) - x(0), 2) + pow(xs(1) - x(1), 2));
            double prev_e_ang = 0.0;
            
            while (e_pos > pos_tol && t < tf) {
                double dx = xs(0) - x(0);
                double dy = xs(1) - x(1);
                e_pos = sqrt(dx*dx + dy*dy);
                
                double theta_des = atan2(dy, dx);
                double e_ang = theta_des - x(2);
                e_ang = atan2(sin(e_ang), cos(e_ang));
                
                double v_des = min(v_max, 0.2 * e_pos);
                double e_vel = v_des - x(3);
                
                double e_ang_dot = (e_ang - prev_e_ang) / dt;
                
                double a = Kp*e_vel + Kd*e_vel/dt + Kp_pos_ff*e_pos;
                double w = Kp_theta*e_ang + Kd_theta*e_ang_dot;
                Vector2d u_nom(a, w);
                
                Vector2d u = solveQP(u_nom);
                
                double x3_new = x(2) + u(1) * dt;
                double x4_new = x(3) + u(0) * dt;
                double x1_new = x(0) + x4_new * cos(x3_new) * dt;
                double x2_new = x(1) + x4_new * sin(x3_new) * dt;
                
                x << x1_new, x2_new, x3_new, x4_new;
                
                state_log.push_back(x);
                Ulog.push_back(u);
                error_log.push_back(Vector2d(e_pos, e_ang));
                
                prev_e_ang = e_ang;
                t += dt;
            }
            
            goal_idx = (goal_idx + 1) % goal_states.size();
        }
    }
    
    void exportData() {
        // Export trajectory
        ofstream traj_file("trajectory.csv");
        traj_file << "x,y,theta,v\n";
        for (const auto& state : state_log) {
            traj_file << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "\n";
        }
        traj_file.close();
        
        // Export control inputs
        ofstream control_file("controls.csv");
        control_file << "acceleration,angular_velocity\n";
        for (const auto& u : Ulog) {
            control_file << u(0) << "," << u(1) << "\n";
        }
        control_file.close();
        
        // Export errors
        ofstream error_file("errors.csv");
        error_file << "position_error,angular_error\n";
        for (const auto& err : error_log) {
            error_file << err(0) << "," << err(1) << "\n";
        }
        error_file.close();
        
        // Export obstacles
        ofstream obs_file("obstacles.csv");
        obs_file << "x,y,radius\n";
        for (const auto& o : obs) {
            obs_file << o.center(0) << "," << o.center(1) << "," << o.radius << "\n";
        }
        obs_file.close();
        
        cout << "Data exported to CSV files!" << endl;
    }
    
    void printResults() {
        cout << "Simulation completed!" << endl;
        cout << "Total timesteps: " << state_log.size() << endl;
        cout << "Final position: (" << x(0) << ", " << x(1) << ")" << endl;
    }
};

int main() {
    ECBFUnicycleController controller;
    controller.run();
    controller.printResults();
    controller.exportData();
    
    return 0;
}