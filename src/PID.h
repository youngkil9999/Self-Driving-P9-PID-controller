#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  int num_step;

  double err;
  double prev_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double p_0;
  double p_1;
  double p_2;
  double dKp;
  double dKi;
  double dKd;

//  int twiddle;

  double tol;
  double p;
  double dp;

  int sum;
  int K_option;

  int numOfstep;
  double best_err;
  int step;
  int TWIDDLE;
  int weight;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  void twiddle(double cte);
  /*
  * Calculate the total PID error.
  */
  double TotalError(double err);
};

#endif /* PID_H */
