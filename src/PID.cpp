#include "PID.h"
#include <time.h>
#include <iostream>
#include <math.h>
#define MIN_STEP 100
#define MAX_STEP 200
#define OFFSET 100
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

    dKp = Kp/4;
    dKi = Ki/4;
    dKd = Kd/4;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    tol = 0.0002;

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

//    cout << "p_error : " << p_error << ", i_error : "<< i_error<< ", d_error :" << d_error << ", prev_d_error " << prev_cte << endl;

    if (num_step > MIN_STEP){
        err += pow(cte, 2);
    }

    num_step++;

}


double PID::TotalError(double err) {

    cout<<"Total Error is :" << err/(num_step - MIN_STEP) << ", num step is : " << num_step << ", error is : "<< err << endl;

    return err/(num_step-MIN_STEP);

}


void PID::twiddle(double cte) {

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

        numOfstep = 0;
        step = 1;

    }

//  Step 1 is add dKp, dKi, dKd setup
    else if (step == 1) {

        if (K_option == 0) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {

                    best_err = err;
                    dKp *= 1.1;
                    step = 0;
                    K_option = 1;
//                    weight += OFFSET;

                    cout << "Step 1, Kp += 1.1" <<endl;
                    cout<< "best err 2 is " << best_err << endl;
                    cout<< "err is " << err << endl;
                } else {

                    Kp -= 2*dKp;
                    step = 2;
//                    sum = 1;
//                    cout << "Step 1, Kp -= 2*dKp" <<endl;
//                    cout<< "best err 3 is " << best_err << endl;
//                    cout<< "err is " << err << endl;
                }

//                cout << "numOfstep is " << numOfstep <<endl;
                numOfstep = 0;
                num_step = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;

            }

        } else if (K_option == 1) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKi *= 1.1;
                    step = 0;
                    K_option = 2;
//                    num_step = 0;
//                    weight += OFFSET;

                    cout << "Step 1, dKi *= 1.1" <<endl;
                    cout<< "best err 4 is " << best_err << endl;
                    cout<< "err is " << err << endl;
                } else {
                    Ki -= 2*dKi;
                    step = 2;
//                    sum = 1;
//                    cout << "Step 1, Ki -= 2*dKi" <<endl;
//                    cout<< "best err 5 is " << best_err << endl;
//                    cout<< "err is " << err << endl;
                }
//                cout << "numOfstep is " << numOfstep <<endl;

                numOfstep = 0;
                num_step = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;

            }

        } else if (K_option == 2) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKd *= 1.1;
                    step = 0;
                    K_option = 0;
//                    num_step = 0;
//                    weight += OFFSET;

                    cout << "Step 1, dKd *= 1.1" <<endl;
                    cout<< "best err 6 is " << best_err << endl;
                    cout<< "err is " << err << endl;
                } else {
                    Kd -= 2*dKd;
//                    sum = 1;

//                    cout << "Step 1, Kd -= 2*dKp" <<endl;
//                    cout<< "best err 7 is " << best_err << endl;
//                    cout<< "err is " << err << endl;
                    step = 2;
                }
//                cout << "numOfstep is " << numOfstep <<endl;

                numOfstep = 0;
                num_step = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;


            }
        }
    }

// Step 2 add last dKp, dKi, dKd
    else if (step == 2){
        if (K_option == 0) {
            UpdateError(cte);
            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKp *= 1.1;
                    num_step = 0;


                    cout << "Step 2, dKp *= 1.1" <<endl;
                    cout<< "best err 8 is " << best_err << endl;
                    cout<< "err is " << err << endl;
                } else {
                    Kp += dKp;
                    dKp *= 0.9;
//                    weight += OFFSET;
//                    sum = 1;

//                    cout << "Step 2, Kp += dKp" <<endl;
//                    cout << "dKp *= 0.9" <<endl;
//                    cout<< "best err 9 is " << best_err << endl;
//                    cout<< "err is " << err << endl;

                }

//                cout << "numOfstep is " << numOfstep <<endl;

                step = 0;
                K_option = 1;
                num_step = 0;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;
                weight += OFFSET;

            }

        } else if (K_option == 1) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKi *= 1.1;
//                    num_step = 0;
//                    weight += OFFSET;

                    cout << "Step 2, dKi *= 1.1" <<endl;
                    cout<< "best err 10 is " << best_err << endl;
                    cout<< "err is " << err << endl;
                } else {
                    Ki += dKi;
                    dKi *= 0.9;
//                    sum = 1;

//                    cout << "Step 2, Ki += dKi" <<endl;
//                    cout << "dKi *= 0.9" <<endl;
//                    cout<< "best err 11 is " << best_err << endl;
//                    cout<< "err is " << err << endl;
                }

//                cout << "numOfstep is " << numOfstep <<endl;

                step = 0;
                K_option = 2;
                num_step = 0;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;
                weight += OFFSET;


            }

        } else if (K_option == 2) {
            UpdateError(cte);

            if (numOfstep > MAX_STEP + weight) {

                err = TotalError(err);

                if (err < best_err) {
                    best_err = err;
                    dKd *= 1.1;
//                    num_step = 0;
//                    weight += OFFSET;

                    cout << "Step 2, dKd *=1.1" <<endl;
                    cout<< "best err 12 is " << best_err << endl;
                    cout<< "err is " << err << endl;

                } else {
                    Kd += dKd;
                    dKd *= 0.9;
//                    sum = 1;

//                    cout << "Step 2, Kd += dKd" <<endl;
//                    cout << "dKd *= 0.9" <<endl;
//                    cout<< "best err 13 is " << best_err << endl;
//                    cout<< "err is " << err << endl;

                }

//                cout << "numOfstep is " << numOfstep <<endl;

                step = 0;
                K_option = 0;
                num_step = 0;
                numOfstep = 0;
                err = 0;
                d_error = 0;
                i_error = 0;
                p_error = 0;
                sum = 1;
                weight += OFFSET;
            }
        }
    }

    numOfstep += 1;



    if ((dKp + dKi + dKd) < tol){
        cout << "Kp OP: " << Kp << "Ki OP: " << Ki << "Kd OP: " << Kd << endl;
        cout << "dKp : " << dKp << " dKi : " << dKi << " dKd : " << dKd << endl;
        TWIDDLE = 2;

        return;
    }

    }

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}