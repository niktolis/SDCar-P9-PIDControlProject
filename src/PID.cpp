#include "PID.h"
#include <iostream>
#include <math.h>
#include <numeric>

using namespace std;


PID::PID(double Kp, double Ki, double Kd, double min, double max, bool use_twiddle) :
  out_min(min), out_max(max), tw_flag(use_twiddle) {

  // Error values initialization
  p_error = 0.0;
  d_error = 0.0;  
  i_error = 0.0;
  cte_prev = 0.0;
    
  // Set Values of the coefficients
  Parameters.Coeff.Kp = Kp;
  Parameters.Coeff.Ki = Ki;
  Parameters.Coeff.Kd = Kd;
  
  // If twiddle is requested initialize its parameters
  if (tw_flag == true) {
    
    InitTwiddle();
  }
}

PID::~PID() {}


void PID::UpdateError(double cte) {


  p_error = cte;
  i_error += cte;
  d_error = cte - cte_prev;
    
  cte_prev = cte;
  
  // If twiddle is requested run the algorithm main function
  if (tw_flag == true) {
    Twiddle();
  }

}


double PID::RunController() {
  
  
  double out;
  
  double p_term = Parameters.Coeff.Kp * p_error;
  double i_term = Parameters.Coeff.Ki * i_error;
  double d_term = Parameters.Coeff.Kd * d_error;
  
  // I part anti-windup so we get rid of accumulated error.
  // This is use because of the saturated output (check below).
  if (i_term > out_max) {
    i_term = out_max;
  }
  else if (i_term < out_min) {
    i_term = out_min;
  }
  else {
    // Do nothing
  }
  
  out = p_term + i_term + d_term;
    
  // Saturation
  if (out > out_max) {
      out = out_max;
  }
  else if (out < out_min) {
      out = out_min;
  }
  else {
    // Do nothing
  }

  return -out;

}

double PID::TotalError() {
    
  double total_error = 0.0;
  
  total_error = pow(p_error, 2);

  return total_error;
}


void PID::InitTwiddle() {
  
  // If it is the real first run of twiddling (all PID gains are 0 then use deltas equal to
  // the ones in the class.
  if (all_of(Parameters.p.begin(), Parameters.p.end(), [](const double & i){return i==0;})) {
    for (int i = 0; i < Parameters.p.size(); i++) {
      dp[i] = 1;
    }
  }
  // Otherwise the deltas should be a fraction of the initialized PID gains.
  else {
    for (int i = 0; i < Parameters.p.size(); i++) {
      dp[i] = 0.25 * Parameters.p[i];
    }
  }
  
  tw_error = 0.0;
  tw_step = 1;
  // Parameter id set to 2 so the first run evaluates the Kp
  param_id = 2;
  best_err = numeric_limits<double>::max();
  increase = false;
  decrease = false;
  
}

void PID::Twiddle() {
  
  
  // Accumulate error for as long as we are in evaluation phase
  if (tw_step % (tw_start + tw_span) > tw_start) {
    tw_error += TotalError();
  }
  
  // When the eval phase finishes
  if (tw_flag && tw_step % (tw_start + tw_span) == 0) {
    
    if (tw_error < best_err) {
      // The total error has been improved. Increase dp for the next time.
      // Move on to the next parameter
      best_err = tw_error;
      if (tw_step !=  tw_start + tw_span) {
        dp[param_id] *= 1.1;
      }
      param_id = (param_id + 1) % 3;
      increase = decrease = false;
    }
    if (!increase && !decrease) {
      // Increase parameter
      Parameters.p[param_id] += dp[param_id];
      increase = true;
    }
    else if (increase && !decrease) {
      // Decrease parameter
      Parameters.p[param_id] -=  2 * dp[param_id];
      decrease = true;
    }
    else {
      // The total error did not improve. Set it back to the original value.
      // Move on to the next parameter.
      Parameters.p[param_id] += dp[param_id];
      dp[param_id] *= 0.9;
      param_id = (param_id + 1) % 3;
      increase = decrease = false;
    }
    // Set the error to 0 before restart the simulation
    tw_error = 0.0;
    // Check if the tolerance of the sum(dp) is reached and interrupt twiddling
    if (accumulate(dp.begin(), dp.end(), 0.0, plus<double>()) <= tw_tol)
    {
      cout << "Twiddling finished! Final Parameters: Kp: " << Parameters.Coeff.Kp << ", Ki: " << Parameters.Coeff.Ki << ", Kd: " << Parameters.Coeff.Kd << endl;
      tw_flag = false;
    }
    // Restart the simulation
    Restart(ws);
  }
  tw_step++;
}


void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  
  // Error values re-initialization
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
  cte_prev = 0.0;
  
  // Twiddling parameters re-initialization
  tw_step = 0;
  tw_error = 0;
  
  // Send reset message to the server
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  
}

void PID::SetServer(uWS::WebSocket<uWS::SERVER> ws) {
  
  this->ws = ws;
}


