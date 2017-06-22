#include "PID.h"
#include <time.h>
#include <iostream>
#include <math.h>
#define MIN_STEP 50
#define MAX_STEP 100
#include <uWS/uWS.h>



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

    dKp = 0.2;
    dKi = 0.2;
    dKd = 0.2;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    tol = 0.2;

    int numOfstep = 0;

    double best_err = 0;
    K_option = 0;


    sum = 0;

    TWIDDLE = 0;
//    twiddle = 0;
    step = 0;



}

void PID::UpdateError(double cte) {

    p_error = cte;

    i_error += cte;

    d_error = cte - prev_cte;

    prev_cte = cte;

    if (num_step > MIN_STEP){
        err += pow(cte, 2);
    }

    num_step++;

}


double PID::TotalError(double err) {

    return err/(num_step-MIN_STEP);

}


void PID::twiddle(double cte) {

//    cout << "option is " << K_option << endl;
// Step 0 is initial Kp, Ki, Kd setup

    if (step == 0) {

        if (K_option == 0) {
            Kp += dKp;
            step = 1;

        } else if (K_option == 1) {
            Ki += dKi;
            step = 1;

        } else if (K_option == 2) {
            Kd += dKd;
            step = 1;
        }
    }

//  Step 1 is add dKp, dKi, dKd setup
    if (step == 1) {

        if (K_option == 0) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKp *= 1.05;
                    step = 0;
                    K_option = 1;

                } else {
                    Kp -= 2*dKp;
                    step = 2;
                }

                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;

            }

        } else if (K_option == 1) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKi *= 1.05;
                    step = 0;
                    K_option = 2;

                } else {
                    Ki -= 2*dKi;
                    step = 2;
                }
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
            }

        } else if (K_option == 2) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKd *= 1.05;
                    step = 0;
                    K_option = 0;
                } else {
                    Kd -= 2*dKd;
                    step = 2;
                }
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;

            }
        }
    }

// Step 2 add last dKp, dKi, dKd
    if (step == 2){
        if (K_option == 0) {
            UpdateError(cte);
            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKp *= 1.1;

                } else {
                    Kp += dKp;
                    dKp *= 0.9;

                }
                step = 0;
                K_option = 1;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
            }

        } else if (K_option == 1) {
            UpdateError(cte);
            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKi *= 1.1;

                } else {
                    Ki *= 0.9;

                }
                step = 0;
                K_option = 2;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
            }

        } else if (K_option == 2) {
            UpdateError(cte);
            if (numOfstep > MAX_STEP) {
                if (err < best_err) {
                    best_err = err;
                    dKd *= 1.1;

                } else {
                    Kd *= 0.9;

                }
                step = 0;
                K_option = 0;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                

            }
        }
    }

    numOfstep += 1;
    cout << "Kp OP: " << Kp << " Ki OP: " << Ki << " Kd OP: " << Kd << endl;


    if ((dKp + dKi + dKd) < tol){
        cout << "Kp OP: " << Kp << "Ki OP: " << Ki << "Kd OP: " << Kd << endl;

        return;
    }


//    UpdateError(cte);

//    if(cal_error == 1){
//        cal_error = 0;
//        if (err < best_err){
//            best_err = err;
//            dp_0 *= 1.1;
//        } else {
//            Kp -= 2 * dp_0;
//        }
//    } else if(cal_error ==2){
//        cal_error = 0;
//        if (err < best_err){
//            best_err = err;
//            dp_1 *= 1.1;
//        } else {
//            Ki -= 2 * dp_1;
//        }
//    } else if(cal_error ==3){
//        cal_error = 0;
//        if (err < best_err){
//            best_err = err;
//            dp_2 *= 1.1;
//        } else {
//            Kd -= 2 * dp_2;
//        }
//    }
//
//    UpdateError(cte);

//    if (err < best_err) {
//        best_err = err;
//        dp[option] += 1.1;
//
//    }
//    else {
//        err = best_err;
//        p[option] -= 2 * dp[option];
//
//    }
//
//
//    if (option == 0){
//        this->dp_0 = dp[option];
//        this->p_0 = p[option];
//    } else if (option == 1) {
//        this->dp_1 = dp[option];
//        this->p_1 = p[option];
//    } else {
//        this->dp_2 = dp[option];
//        this->p_2 = p[option];
//    }
//
//    for (int i = 0; i < 3; ++i) {
//        sum += dp[i];
//    }
//
//    cout << "Sum is " << sum << endl;
//
//    if (sum <= 0.2){
//
//        this->Kp = p_0;
//        this->Ki = p_1;
//        this->Kd = p_2;
//
//    } else {
//
//        cout << " Not ready to optimize the PID K values" << endl;
//        cout << "Kp is " << Kp << "Ki is " << Ki << "Kd is " << Kd <<endl;
//
//    }


//    for (int idx = 0; idx<3;idx++ ) {
//
//        p[idx] += dp[idx];
//
//        cout << p[0] << ' ' << p[1] << ' ' << p[2] << endl;
//
//        UpdateError(cte);
//    }
//
//
//    if (err < best_err){
//        best_err = err;
//        dp[idx] *= 1.1;
//    }
//
//            else {
//                p[idx] -= 2 * dp[idx];
//                UpdateError(cte);
//
//                if (err < best_err){
//                    best_err = err;
//                    dp[idx] *= 1.1;
//                }
//
//                else{
//                    p[idx] += dp[idx];
//                    dp[idx] *= 0.9;
//                }
//            }
//
//        }
//
//        sum = dp[0] + dp[1] + dp[2];
//
//        Kp = p[0];
//        Ki = p[1];
//        Kd = p[2];
//
//        cout<< err << ' ' << best_err << ' ' << sum << ' ' << tol << endl;
//
//    } while (sum > tol);
//
//
//
//    cout<<Kp << ' ' << Ki << ' ' << Kd << endl;


}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"restart\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}