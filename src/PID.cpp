#include "PID.h"
#include <time.h>


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

}

double PID::TotalError() {


//    return Kp, Ki, Kd;
}


void twiddle(int tol = 0.2){

//    p = [0, 0, 0];
//    dp = [1, 1, 1];
//    robot = make_robot()
//
//    it = 0
//    while sum(dp) > tol:
//            print("Iteration {}, best error = {}".format(it, best_err))
//    for i in range(len(p)):
//    p[i] += dp[i]
//    robot = make_robot()
//    x_trajectory, y_trajectory, err = run(robot, p)
//
//    if err < best_err:
//            best_err = err
//    dp[i] *= 1.1
//    else:
//    p[i] -= 2 * dp[i]
//    robot = make_robot()
//    x_trajectory, y_trajectory, err = run(robot, p)
//
//    if err < best_err:
//            best_err = err
//    dp[i] *= 1.1
//    else:
//    p[i] += dp[i]
//    dp[i] *= 0.9
//    it += 1

    return Kp, Ki, Kd;
}