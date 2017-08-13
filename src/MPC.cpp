#include "MPC.h"
#include <cppad/ipopt/solve.hpp>

//
// MPC class definition implementation.
//
MPC::MPC() {
    // MPC is initialized here
    size_t N = 10;         // timestep count
    double dt = .1;        // timestep duration, s
    double Lf = 2.67;      // Distance form vehicle's front to it's center of gravity
    double ref_v = 70;     // reference speed, mph
    double latency = 100;  // actuator's control command latency, ms
    int poly_order = 3;    // the order of fitted polynomial
    
    MPC(N, dt, Lf, ref_v, latency, poly_order);
}

MPC::MPC(size_t n, double dt, double Lf, double ref_v, double latency, int poly_order) {
    
    this->n_ = n;
    this->dt_ = dt;
    this->Lf_ = Lf;
    this->ref_v_ = ref_v;
    this->cost_weights_ = {.5, 500, .4, 1, 1, .003, .03, 500, 1};
    this->latency_ = latency;
    this->poly_order_ = poly_order;
}

void MPC::operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;
    
    ADdouble curr_cte = 0;
    ADdouble curr_epsi = 0;
    ADdouble curr_v = 0;
    ADdouble curr_delta = 0;
    ADdouble curr_accel = 0;
    ADdouble next_delta = 0;
    ADdouble next_accel = 0;
    
    for (int t = 0; t < n_; t++) {
        
        curr_cte = vars[get_cte_start() + t];
        curr_epsi = vars[get_epsi_start() + t];
        curr_v = vars[get_v_start() + t];
        
        /// The part of the cost based on the reference state.
        /// Expresses general concern about cte, epsi and maintaining speed
        fg[0] += cost_weights_[0] * CppAD::pow(curr_cte, 2);
        fg[0] += cost_weights_[1] * CppAD::pow(curr_epsi, 2);
        fg[0] += cost_weights_[2] * CppAD::pow(curr_v - ref_v_, 2);
        
        /// Usage of actuators.
        if (t < n_ - 1) {
            
            curr_delta = vars[get_delta_start() + t];
            curr_accel = vars[get_a_start() + t];
            
            /// Minimizes the use of actuators.
            fg[0] += cost_weights_[3] * CppAD::pow(curr_delta, 2);
            fg[0] += cost_weights_[4] * CppAD::pow(curr_accel, 2);
            
            /// Adds additional cost for co-dependence of curent speed, acceleration and cte/epsi
            fg[0] += cost_weights_[5] * CppAD::pow(curr_accel * curr_v * curr_cte, 2);
            fg[0] += cost_weights_[6] * CppAD::pow(curr_accel * curr_v * curr_epsi, 2);

        }
        
        if (t < n_ - 2) {
            
            next_delta = vars[get_delta_start() + t + 1];
            next_accel = vars[get_a_start() + t + 1];
            
            /// Minimizes the value gap between sequential actuations.
            fg[0] += cost_weights_[7] * CppAD::pow(next_delta - curr_delta, 2);
            fg[0] += cost_weights_[8] * CppAD::pow(next_accel - curr_accel, 2);
        }
    }
    
    copy_with_offset(fg, vars, starts(), 1);
    
    for (int t = 1; t < n_; ++t) {
        /// The state at time t+1 .
        State<CppAD::AD<double>> state_1 = set_state<CppAD::AD<double>>(t, vars);
        
        /// The state at time t.
        State<CppAD::AD<double>> state_0 = set_state<CppAD::AD<double>>(t-1, vars);
        
        update_fg(fg, vars, t, state_1, state_0);
    }
}

MPC::~MPC() {}

std::vector<size_t> MPC::starts() {
    return {get_x_start(), get_y_start(), get_psi_start(), get_v_start(), get_cte_start(), get_epsi_start()};
}

MPC::ADdouble MPC::dx(ADdouble x_1, ADdouble x_0, ADdouble v0, ADdouble psi0) {
    return x_1 - (x_0 + v0 * CppAD::cos(psi0) * dt_);
}
MPC::ADdouble MPC::dy(ADdouble y_1, ADdouble y_0, ADdouble v0, ADdouble psi0) {
    return y_1 - (y_0 + v0 * CppAD::sin(psi0) * dt_);
}
MPC::ADdouble MPC::dpsi(ADdouble psi_1, ADdouble psi_0, ADdouble v0, ADdouble delta0) {
    return psi_1 - (psi_0 - v0 / Lf_ * delta0 * dt_);
}
MPC::ADdouble MPC::dv(ADdouble v_1, ADdouble v_0, ADdouble a) {
    return v_1 - (v_0 + a * dt_);
}
MPC::ADdouble MPC::dcte(ADdouble cte_1, ADdouble f0, ADdouble y0, ADdouble v0, ADdouble epsi0) {
    return  cte_1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt_);
}
MPC::ADdouble MPC::depsi(ADdouble epsi1, ADdouble psi0, ADdouble psides0, ADdouble v0, ADdouble delta0) {
    return epsi1 - ((psi0 - psides0) + v0 / Lf_ * delta0 * dt_);
}


void MPC::update_fg(ADvector &fg, const ADvector &vars, int t, State<ADdouble> &state_1, State<ADdouble> &state_0) {
    
    ADdouble delta0 = vars[get_delta_start() + t - 1];
    ADdouble a0 = vars[get_a_start() + t - 1];
    
    ADdouble f0 = 0;
    ADdouble df0 = 0;
    
    /// Computes f(x) at state_0 and it's derivative
    for (int i = 0; i < coeffs_.size(); ++i) {
        f0 += coeffs_[i] * pow(state_0.x, i);
        
        if (i > 0) {
            df0 += i * coeffs_[i] * pow(state_0.x, i-1);
        }
    }
    
    /// Computes desired psi as the arctangent of the derivative.
    ADdouble psides0 = CppAD::atan(df0);
    
    /// Updates constraints
    fg[1 + get_x_start() + t] = dx(state_1.x, state_0.x, state_0.v, state_0.psi);
    fg[1 + get_y_start() + t] = dy(state_1.y, state_0.y, state_0.v, state_0.psi);
    fg[1 + get_psi_start() + t] = dpsi(state_1.psi, state_0.psi, state_0.v, delta0);
    fg[1 + get_v_start() + t] = dv(state_1.v, state_0.v, a0);
    fg[1 + get_cte_start() + t] = dcte(state_1.cte, f0, state_0.y, state_0.v, state_0.epsi);
    fg[1 + get_epsi_start() + t] = depsi(state_1.epsi, state_0.psi, psides0, state_0.v, delta0);
}

void MPC::copy_with_offset(ADvector &target, const ADvector &source, std::vector<size_t> filter, int offset) {
    for (size_t start: filter) {
        target[offset + start] = source[start];
    }
}

template<class T>
State<T> MPC::set_state(Eigen::VectorXd vector) {
    assert(vector.size() == state_dimension_);
    
    State<T> state;
    
    state.x = vector[0];
    state.y = vector[1];
    state.psi = vector[2];
    state.v = vector[3];
    state.cte = vector[4];
    state.epsi = vector[5];
    
    return state;
}

template<class T>
State<T> MPC::set_state(int t, const ADvector& vars) {
    State<T> state;
    
    state.x = vars[MPC::get_x_start() + t];
    state.y = vars[MPC::get_y_start() + t];
    state.psi = vars[MPC::get_psi_start() + t];
    state.v = vars[MPC::get_v_start() + t];
    state.cte = vars[MPC::get_cte_start() + t];
    state.epsi = vars[MPC::get_epsi_start() + t];
    
    return state;
}

template<class T>
void MPC::update_by_state(MPC::Dvector &vector, State<T> &state, std::vector<size_t> filter) {
    
    std::vector<T> state_vals = state_values(state);
    
    for (int i = 0; i < filter.size(); ++i) {
        vector[filter[i]] = state_vals[i];
    }
}

template<class T>
std::vector<T> MPC::state_values(State<T> state) {
    return {state.x, state.y, state.psi, state.v, state.cte, state.epsi};
}

/// The solver takes all the state variables and actuator
/// variables in a singular vector. Thus, we should establish
/// when one variable starts and another ends to make life easier.
size_t MPC::get_x_start() { return 0; }
size_t MPC::get_y_start() { return n_; }
size_t MPC::get_psi_start() { return n_ * 2; }
size_t MPC::get_v_start() { return n_ * 3; }
size_t MPC::get_cte_start() { return n_ * 4; }
size_t MPC::get_epsi_start() { return n_ * 5; }
size_t MPC::get_delta_start() { return n_ * 6; }
size_t MPC::get_a_start() { return n_ * 7 - 1; }

void MPC::set_next_vals() {
    
    next_x_vals_.clear();
    next_y_vals_.clear();
    
    double poly_inc = 2;
    int num_points = 20;
    for(int i = 1; i < num_points; ++i) {
        next_x_vals_.push_back(poly_inc*i);
        next_y_vals_.push_back(polyeval(coeffs_, poly_inc*i));
    }
}

void MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
    
    mpc_x_vals_.clear();
    mpc_y_vals_.clear();
    this->coeffs_ = coeffs;
    
    State<double> state = set_state<double>(x0);
    
    /// number of independent variables
    /// N timesteps -> N - 1 actuations
    /// e. g., 25 timesteps, 6 dimensions per timestep
    /// 24 actuator tuples, each containing 2 values: (delta, accel)
    /// total of 25 * 6 + 24 * 2 = 198 elements.
    size_t n_vars = n_ * state_dimension_ + (n_ - 1) * 2;
    /// Number of constraints (25 * 6 = 150)
    size_t n_constraints = n_ * state_dimension_;
    
    /// Initial value of the independent variables.
    /// Should be 0 except for the initial values.
    Dvector vars(n_vars);
    
    for (int i = 0; i < n_vars; ++i) {
        vars[i] = 0.0;
    }
    /// Set the initial variable values
    update_by_state(vars, state, starts());
    
    /// Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    /// Set all non-actuators upper and lowerlimits
    /// to the max negative and positive values.
    for (int i = 0; i < get_delta_start(); ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    
    /// The upper and lower limits of delta are set to -25 and 25
    /// degrees (values in radians).
    /// NOTE: Feel free to change this to something else.
    for (int i = get_delta_start(); i < get_a_start(); ++i) {
        vars_lowerbound[i] = deg2rad(-25);
        vars_upperbound[i] = deg2rad(25);
    }
    
    /// Acceleration/decceleration upper and lower limits.
    /// NOTE: Feel free to change this to something else.
    for (int i = get_a_start(); i < n_vars; ++i) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }
    
    /// Lower and upper limits for constraints
    /// All of these should be 0...
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    
    /// ...except the initial state indices.
    update_by_state(constraints_lowerbound, state, starts());
    update_by_state(constraints_upperbound, state, starts());
    
    /// options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    
    /// place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    /// solve the problem
    CppAD::ipopt::solve<Dvector, MPC>(options, vars, vars_lowerbound,
                                      vars_upperbound, constraints_lowerbound,
                                      constraints_upperbound, *this, solution);
    
    steer_ = solution.x[get_delta_start()]/deg2rad(25);
    accel_ = solution.x[get_a_start()];
    
    /// fill minimum cost trajectory points
    for (int i = 0; i < n_ - 1; ++i) {
        
        mpc_x_vals_.push_back(solution.x[get_x_start() + i + 1]);
        mpc_y_vals_.push_back(solution.x[get_y_start() + i + 1]);
    }
    
    /// fill the reference points in vehicles coordinate space
    set_next_vals();
}
