#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double cte_prev;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID(double Kp, double Ki, double Kd);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * PID Controller main function
  */
  double Controller();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
