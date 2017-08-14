#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "MPC.h"
#include "json.hpp"
#include "helper.h"

// for convenience
using json = nlohmann::json;

int main(int argc, char* argv[]) {
    uWS::Hub h;
    
    // MPC is initialized here
    int N = 10;             // timestep count
    double dt = .1;         // timestep duration, s
    double Lf = 2.67;       // Distance form vehicle's front to it's center of gravity
    double ref_v = 70;     // reference speed, mph
    double latency = 100;   // actuator's control command latency, ms
    int poly_order = 3;     // the order of fitted polynomial
    MPC mpc = MPC(N, dt, Lf, ref_v, latency, poly_order);
    
    /// Params may be specified as arguments.
    if (argv[1]) {
        std::vector<double> weights(9);
        
        mpc.ref_v_ = atof(argv[1]);
        
        for (size_t i = 0; i < weights.size(); ++i) {
            weights[i] = atof(argv[i + 2]);
        }
        
        mpc.cost_weights_ = weights;
    }
    
    std::cout << "ref v: " << mpc.ref_v_ << std::endl;
    for (int i = 0; i < mpc.cost_weights_.size(); ++i) {
        std::cout << mpc.cost_weights_[i] << " ";
    }
    std::cout << std::endl;
    
    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        std::string sdata = std::string(data).substr(0, length);
//        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            std::string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    std::vector<double> ptsx = j[1]["ptsx"];
                    std::vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    
                    double steer_value = j[1]["steering_angle"];
                    double accel = j[1]["throttle"];
                    
                    /// Convert points to vehicle's coordinate space
                    convert_space(ptsx, ptsy, px, py, psi);
                    
                    
                    /// Eigen::VectorXd versions of points vectors
                    Eigen::VectorXd ptsx_e = vec2eigen(ptsx);
                    Eigen::VectorXd ptsy_e = vec2eigen(ptsy);
                    
                    /// Computing parameters at T+latency.
                    double vms = v * mph2ms();
                    
                    /// This isn't strictly true, but there's no other measure of acceleration
                    /// except the throttle value.
                    /// Though, completely omitting this step (condidering speed inchanged) also works well.
                    double vms1 = vms + accel * (mpc.latency_ / 1000);
                    
                    /// Depending on steer_value, psi might have become non-zero during the period of latency
                    psi = vms1 / mpc.Lf_ * -steer_value * (mpc.latency_ / 1000);
                    
                    /// Segment length travelled during the period of latency
                    double L = vms * (mpc.latency_ / 1000) + .5 * accel * pow(mpc.latency_ / 1000, 2);
                    
                    /// Derived geometrically. (L / psi) is a curvature radius: https://en.wikipedia.org/wiki/Circular_segment
                    px = std::abs(psi) > 0.0000001 ? (L / psi) * sin(std::abs(psi)) : L;
                    py = px * tan(psi);
                    
                    /// Solver will have the speed back in mph. This is sort of a bug,
                    /// which has been seemingly sorted out by the CppAD::ipopt::solve.
                    /// I did try to fix that feeding speed in m/s, but couldn't obtain the same smooth
                    /// behavior as with mph.
                    v = vms1 / mph2ms();
                    
                    Eigen::VectorXd coeffs = polyfit(ptsx_e, ptsy_e, mpc.poly_order_);
                    
                    double cte = polyeval(coeffs, px) - py;
                    
                    double dy = derivative(coeffs, px);
                    double epsi = 0 - atan(dy);

                    Eigen::VectorXd state(6);
                    state << px, py, psi, v, cte, epsi;
                    
                    mpc.Solve(state, coeffs);
                    
                    /// Prepare JSON message for the simulator environment
                    json msgJson;
                    
                    /// New values for the actuators controls
                    msgJson["steering_angle"] = mpc.steer_;
                    msgJson["throttle"] = mpc.accel_;
                    
                    /// points of minimum cost trajectory returned from the solver (green line)
                    msgJson["mpc_x"] = mpc.mpc_x_vals_;
                    msgJson["mpc_y"] = mpc.mpc_y_vals_;
                    
                    /// Reference points in the vehicle's coordinate system (yellow line)
                    msgJson["next_x"] = mpc.next_x_vals_;
                    msgJson["next_y"] = mpc.next_y_vals_;
                    
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    const unsigned long timeToSleep = static_cast<double>(mpc.latency_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(timeToSleep));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
