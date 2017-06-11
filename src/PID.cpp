#include "PID.h"
#include <time.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = 0;
    i_error = 0;
    d_error = 0;

}

void PID::UpdateError(double cte) {

    double prev_cte;

    p_error = cte;

    i_error += cte;

    d_error = cte - prev_cte;

    prev_cte = cte;

//    double total_err =
}

double PID::TotalError() {

//    return Kp, Ki, Kd;
}


//void twiddle(int tol = 0.2){
//
//    double p[] = {0, 0, 0};
//    double dp[] = {1, 1, 1};
////    robot = make_robot()
////
//    int it = 0;
//    int sum = 0;
//
//    do {
//
//        for (int idx = 0; idx<sizeof(dp);idx++ ){
//            sum += dp[idx];
//            p[idx] = dp[idx];
//            robot = make_robot();
//            x_trajectory, y_trajectory, err = run(robot, p);
//
//            if (err < best_err){
//                best_err = err;
//                dp[idx] *= 1.1;
//            }
//
//            else {
//                p[idx] -= 2 * dp[idx];
////                robot = make_robot()
////                x_trajectory, y_trajectory, err = run(robot, p)
//
//                if (err < best_err){
//                best_err = err;
//                dp[idx] *= 1.1;
//                }
//
//                else{
//                p[idx] += dp[idx];
//                dp[idx] *= 0.9;
//                }
//            }
//        }
//    } while (sum > tol);
//}