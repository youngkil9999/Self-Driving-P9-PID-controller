#include "PID.h"
#include <time.h>
#include <iostream>
#include <math.h>
#define MIN_STEP 300
#define MAX_STEP 1000
#define OFFSET 400
#define H_Limit 1.5
#define L_Limit -1.5
#include <uWS/uWS.h>
#include <stdlib.h>
#include <stdio.h>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

//    dKp = 1;
//    dKi = 1;
//    dKd = 1;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    tol = 0.002;

    int numOfstep = 0;

    K_option = 0;

    sum = 0;

    TWIDDLE = 0;
    step = 0;
    weight = 0;
    prev_cte = 0;

}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;

    p_error = cte;

    i_error += cte;

//    d_error = p_error - prev_cte;
//    prev_cte = cte;

    if (num_step > MIN_STEP) {
        err += pow(cte, 2);
    }

    num_step++;

}


double PID::TotalError() {

    return -Kp * p_error - Ki * i_error - Kd * d_error;

}


void PID::twiddle(double cte) {

//    cout << cte <<endl;

//    if (step == 0) {
//
//        if (K_option == 0) {
//            Kp += dKp;
////            cout << "Step 0, Kp += dKp" <<endl;
//        } else if (K_option == 1) {
//            Ki += dKi;
////            cout << "Step 0, Ki += dKi" <<endl;
//        } else if (K_option == 2) {
//            Kd += dKd;
////            cout << "Step 0, Kd += dKd" <<endl;
//        }
//
//        numOfstep = 0;
//        step = 1;
//
//    }
//
////  Step 1 is add dKp, dKi, dKd setup
//    else if (step == 1) {
//
//        if (K_option == 0) {
//            UpdateError(cte);
//
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit )&& err > 0
//            if ( (numOfstep > MAX_STEP || (cte > H_Limit || cte < L_Limit )) && err >0 ) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//
//                    best_err = err;
//                    dKp *= 1.1;
//                    step = 0;
//                    K_option = 1;
//
//                } else {
//
//                    Kp -= 2*dKp;
//                    step = 2;
//
//                }
//
//                sum = 1;
//
//            }
//
//        } else if (K_option == 1) {
//            UpdateError(cte);
//
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit )&& err > 0
//            if ((numOfstep > MAX_STEP|| (cte > H_Limit || cte < L_Limit )) && err >0) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//                    best_err = err;
//                    dKi *= 1.1;
//                    step = 0;
//                    K_option = 2;
//
//                } else {
//                    Ki -= 2*dKi;
//                    step = 2;
//
//                }
//
//                sum = 1;
//
//            }
//
//        } else if (K_option == 2) {
//            UpdateError(cte);
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit)&& err > 0
//            if ((numOfstep > MAX_STEP|| (cte > H_Limit || cte < L_Limit )) && err >0) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//                    best_err = err;
//                    dKd *= 1.1;
//                    step = 0;
//                    K_option = 0;
//
//                } else {
//                    Kd -= 2*dKd;
//                    step = 2;
//                }
//
//                sum = 1;
//
//            }
//        }
//    }
//
//// Step 2 add last dKp, dKi, dKd
//    else if (step == 2){
//
//        if (K_option == 0) {
//            UpdateError(cte);
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit)&& err > 0
//            if ((numOfstep > MAX_STEP|| (cte > H_Limit || cte < L_Limit )) && err >0) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//                    best_err = err;
//                    dKp *= 1.1;
//
//                } else {
//                    Kp += dKp;
//                    dKp *= 0.9;
//
//                }
//
//                step = 0;
//                K_option = 1;
//                sum = 1;
//                weight += OFFSET;
////                best_err = 0;
//
//            }
//
//        } else if (K_option == 1) {
//            UpdateError(cte);
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit)&& err > 0
//            if ((numOfstep > MAX_STEP|| (cte > H_Limit || cte < L_Limit )) && err >0) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//                    best_err = err;
//                    dKi *= 1.1;
//
//                } else {
//                    Ki += dKi;
//                    dKi *= 0.9;
//
//                }
//
//                step = 0;
//                K_option = 2;
//                sum = 1;
//                weight += OFFSET;
////                best_err = 0;
//            }
//
//        } else if (K_option == 2) {
//            UpdateError(cte);
////            numOfstep > MAX_STEP + weight ||
////            (cte > H_Limit || cte < L_Limit)&& err > 0
//            if ((numOfstep > MAX_STEP|| (cte > H_Limit || cte < L_Limit )) && err >0) {
//                err = TotalError(err);
//
//                if (err < best_err) {
//                    best_err = err;
//                    dKd *= 1.1;
//                } else {
//                    Kd += dKd;
//                    dKd *= 0.9;
//                }
//
//                step = 0;
//                K_option = 0;
//                sum = 1;
//                weight += OFFSET;
////                best_err = 0;
//            }
//        }
//    }
//
//    numOfstep += 1;
//
//
//
//    if ((dKp + dKi + dKd) < tol){
//        cout << "Kp OP: " << Kp << "Ki OP: " << Ki << "Kd OP: " << Kd << endl;
//        cout << "dKp : " << dKp << " dKi : " << dKi << " dKd : " << dKd << endl;
//        TWIDDLE = 2;
//
//        return;
//    }

    }

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}