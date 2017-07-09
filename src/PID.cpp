#include "PID.h"
#include <time.h>
#include <iostream>
#include <math.h>
#define MIN_STEP 100
#define MAX_STEP 6000
#define OFFSET 500
#define H_Limit 1.8
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

void PID::Init(double Kp_, double Ki_, double Kd_) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

//    dKp = 0.000760891;
//    dKi = 0.00000929978;
//    dKd = 0.000126293;

    dKp = Kp/4;
    dKi = Ki/4;
    dKd = Kd/4;

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

    p_error = cte;

    i_error += cte;

    d_error = p_error - prev_cte;

    prev_cte = cte;

    if (num_step > MIN_STEP) {
        err += pow(cte, 2);
    }

    num_step++;

}


double PID::TotalError(double err) {
//    cout << "Twiddle 1" << endl;
    cout<<" Total Error is :" << err/(num_step - MIN_STEP) << ", num step is : " << num_step << ", error is : "<< err << endl;
    cout<< "best err is : " << best_err << "Time period : " << num_step + weight << endl;
    cout << "Kp : " << Kp << " Ki : " << Ki << " Kd : " << Kd << endl;
    cout << "dKp : " << dKp << " dKi : " << dKi << " dKd : " << dKd << endl;
//    cout<< "best err is : " << best_err << "Time period : " << num_step + weight << endl;

    return err/(num_step-MIN_STEP);

}


void PID::twiddle(double cte) {

//    cout << cte <<endl;

    if (step == 0) {

        if (K_option == 0) {
            Kp += dKp;
//            cout << "Step 0, Kp += dKp" <<endl;
        } else if (K_option == 1) {
            Ki += dKi;
//            cout << "Step 0, Ki += dKi" <<endl;
        } else if (K_option == 2) {
            Kd += dKd;
//            cout << "Step 0, Kd += dKd" <<endl;
        }

//        numOfstep = 0;
        step = 1;
        sum = 1;


    }

//  Step 1 is add dKp, dKi, dKd setup
    else if (step == 1) {

        if (K_option == 0) {

            UpdateError(cte);

//            numOfstep > MAX_STEP + weight ||
//            (cte > H_Limit || cte < L_Limit )&& err > 0
//            || (fabs(cte) > H_Limit )

            if ((numOfstep > MAX_STEP + weight) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKp *= 1.1;
                    step = 0;
                    K_option = 1;
                    weight += OFFSET;
                    sum = 2;

                } else {

                    Kp -= 2*dKp;
                    step = 2;
                    sum = 1;

                }

            }

        } else if (K_option == 1) {
            UpdateError(cte);

            if ((numOfstep > MAX_STEP + weight) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKi *= 1.1;
                    step = 0;
                    K_option = 2;
                    weight += OFFSET;
                    sum = 2;


                } else {
                    Ki -= 2*dKi;
                    step = 2;
                    sum = 1;

                }

            }

        } else if (K_option == 2) {
            UpdateError(cte);

            if ((numOfstep > MAX_STEP + weight ) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKd *= 1.1;
                    step = 0;
                    K_option = 0;
                    weight += OFFSET;
                    sum = 2;

                } else {
                    Kd -= 2*dKd;
                    step = 2;
                    sum = 1;

                }
            }
        }
    }

// Step 2 add last dKp, dKi, dKd
    else if (step == 2){

        if (K_option == 0) {
            UpdateError(cte);

            if ((numOfstep > MAX_STEP + weight ) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKp *= 1.1;
                    weight += OFFSET;
                    sum = 2;

                } else {
                    Kp += dKp;
                    dKp *= 0.9;
                    sum = 1;

                }

                step = 0;
                K_option = 1;

            }

        } else if (K_option == 1) {
            UpdateError(cte);

            if ((numOfstep > MAX_STEP + weight) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKi *= 1.1;
                    weight += OFFSET;
                    sum = 2;


                } else {
                    Ki += dKi;
                    dKi *= 0.9;
                    sum = 1;

                }

                step = 0;
                K_option = 2;
            }

        } else if (K_option == 2) {
            UpdateError(cte);

            if ((numOfstep > MAX_STEP + weight) && err > 0) {
                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKd *= 1.1;
                    weight += OFFSET;
                    sum = 2;

                } else {
                    Kd += dKd;
                    dKd *= 0.9;
                    sum = 1;

                }

                step = 0;
                K_option = 0;

            }
        }
    }

    numOfstep += 1;



    if ((dKp + dKi + dKd) < tol){
        cout << "Kp OP: " << Kp << "Ki OP: " << Ki << "Kd OP: " << Kd << endl;
        cout << "dKp : " << dKp << " dKi : " << dKi << " dKd : " << dKd << endl;
        cout << "Twiddle 2" << endl;

        TWIDDLE = 2;

        return;
    }

    }

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}