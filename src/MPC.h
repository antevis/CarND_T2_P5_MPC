#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "helper.h"

/** Represents the vehicle's state. values may be both CppADD::AD<double> and double.
 * Hence, leveraging generics.
 */
template <typename T>
struct State {
    T x;
    T y;
    T psi;
    T v;
    T cte;
    T epsi;
};

class MPC {
public:
    typedef CppAD::AD<double> ADdouble;
    typedef CPPAD_TESTVECTOR(ADdouble) ADvector;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    double Lf_;                          // front to CoG distance
    double ref_v_;                       // reference speed
    std::vector<double> cost_weights_;   // cost weights
    double latency_;                     // latency, ms
    int poly_order_;                     // order of the polynomial
    
    double steer_;                       // computed steering control angle in RADIANS [-...+25*pi/180]
    double accel_;                       // computed accelerator control value [-1...1]
    
    // points of minimum cost trajectory returned from the solver (green line)
    std::vector<double> mpc_x_vals_;
    std::vector<double> mpc_y_vals_;
    
    // Reference points in the vehicle's coordinate system (yellow line)
    std::vector<double> next_x_vals_;
    std::vector<double> next_y_vals_;
    
    Eigen::VectorXd coeffs_;             // polynomial coefficients
    
    /** Constructors/Destructor */
    MPC();
    MPC(size_t n, double dt, double Lf, double ref_v, double latency, int poly_order);
    virtual ~MPC();
    
    /** Solve the model given an initial state and polynomial coefficients.
     * @param state: the current state of the vehicle
     * @param coeffs: the polynomial coefficients
     */
    void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
    /** Need this for CppAD::ipopt::solve to work
     * @param fg: vector of constraints
     * @param vars: vector of variables
     */
    void operator()(ADvector& fg, const ADvector& vars);
    
private:
    
    /** Updates vector of constraints according to the vector of vars and states at
     * timestep t and t+1
     * @param fg: vector of constraints
     * @param vars: vector of variables
     * @param t: timestep
     * @param state_1: state at time t+1
     * @param state_0: state at time t
     */
    void update_fg(ADvector& fg, const ADvector& vars, int t,
                   State<ADdouble>& state_1,
                   State<ADdouble>& state_0);
    
    /** Differecnces of respective constituents of the vehicle's state according to
     * kinematic model. Used for setting constraints
     */
    ADdouble dx(ADdouble x_1, ADdouble x_0, ADdouble v0, ADdouble psi0);
    ADdouble dy(ADdouble y_1, ADdouble y_0, ADdouble v0, ADdouble psi0);
    ADdouble dpsi(ADdouble psi_1, ADdouble psi_0, ADdouble v0, ADdouble delta0);
    ADdouble dv(ADdouble v_1, ADdouble v_0, ADdouble a0);
    ADdouble dcte(ADdouble cte_1, ADdouble f0, ADdouble y0, ADdouble v0, ADdouble epsi0);
    ADdouble depsi(ADdouble epsi1, ADdouble psi0, ADdouble psides0, ADdouble v0, ADdouble delta0);
    
    /** Copy one vector to another
     * @param target: target vector of type CppAD::AD<double>
     * @param source: source vector of type CppAD::AD<double>
     * @param filter: a vector of indices of interest
     * @param offset: a shift determining the indices in th target vector
     */
    void copy_with_offset(ADvector& target, const ADvector& source, std::vector<size_t> filter, int offset);
    
    /** Returns start indices of the state constituents as as vector
     * @return: vector of type size_t
     */
    std::vector<size_t> starts();
    
    /** Returns state values as a vector
     * @return: vector of generic type
     */
    template<class T>
    std::vector<T> state_values(State<T> state);
    
    /** Creates state from a given vector
     * @param vector: vector of type Eigen::VectorXd
     */
    template<class T>
    State<T> set_state(Eigen::VectorXd vector);
    
    /** Creates state for a given timestep index from a vector of fariables
     * @param t: timestep  index
     * @param vars: vector of variables (that one long vector)
     */
    template<class T>
    State<T> set_state(int t, const ADvector& vars);
    
    /** Updates vector according to a given state and a filter of indices
     * @param vector: a vector to update
     * @param state: source of values fo updating of type State<T>
     * @param filter: indices of the values in the target values to be updated
     */
    template<class T>
    void update_by_state(Dvector& vector, State<T>& state, std::vector<size_t> filter);
    
    
    /**
     * getters of the state components and actuators controls
     * within the variables
     */
    size_t get_x_start();
    size_t get_y_start();
    size_t get_psi_start();
    size_t get_v_start();
    size_t get_cte_start();
    size_t get_epsi_start();
    size_t get_delta_start();
    size_t get_a_start();
    
    /** Fills the reference points in vehicle's coordinate space*/
    void set_next_vals();
    

    size_t n_;                           // timesteps count
    double dt_;                          // timestep duration, s
    const size_t state_dimension_ = 6;   // state dimensionality
};

#endif /* MPC_H */
