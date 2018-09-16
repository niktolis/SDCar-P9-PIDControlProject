#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
  cte_prev = 0.0;

}

PID::~PID() {}


void PID::UpdateError(double cte) {

  p_error = cte;
  i_error += cte;
  d_error = cte - cte_prev;
  cte_prev = cte;

}


double PID::Controller() {
  
  double p_term = -Kp * p_error;
  double i_term = -Ki * i_error;
  double d_term = -Kd * d_error;

  return p_term + i_term + d_term;

}

double PID::TotalError() {

  return 0.0;
}

