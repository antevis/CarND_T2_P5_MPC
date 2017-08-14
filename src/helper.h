//
//  helper.h
//  mpc
//
//  Created by Ivan Kazakov on 10/08/2017.
//

#ifndef helper_h
#define helper_h

#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

inline double deg2rad(double x) { return x * M_PI / 180; }
inline double rad2deg(double x) { return x * 180 / M_PI; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
inline std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
inline double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

inline void convert_space(std::vector<double>& ptsx, std::vector<double>& ptsy, double px, double py, double psi) {
    for (int i = 0; i < ptsx.size(); ++i) {
        double x_offset = ptsx[i] - px;
        double y_offset = ptsy[i] - py;
        
        ptsx[i] = x_offset * cos(-psi) - y_offset * sin(-psi);
        ptsy[i] = x_offset * sin(-psi) + y_offset * cos(-psi);
    }
}

inline Eigen::VectorXd vec2eigen(std::vector<double> source) {
    return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(source.data(), source.size());
}

inline double mph2ms() {
    const double mPerMile = 1609.34;
    const double sPerHour = 3600;
    
    return mPerMile / sPerHour;
}

inline double derivative(Eigen::VectorXd coeffs, double x) {
    double dy = 0;
    for (int i = 1; i < coeffs.size(); ++i) {
        dy += i * coeffs[i] * pow(x, i-1);
    }
    
    return dy;
}


#endif /* helper_h */
