#ifndef PID_H
#define PID_H

#include <array>
#include <uWS/uWS.h>

class PID {
private:
  
  /*
  * Errors
  */
  double p_error;   /*!< Proportional error */
  double i_error;   /*!< Differential error */
  double d_error;   /*!< Integral error */
  double cte_prev;  /*!< Cross Track Error past value */

  /*
  * Parameter Union
  */
  union ParameterTag {
    
    struct CoeffsTag {
      
      double Kp;  /*!< Proportional term gain */
      double Ki;  /*!< Integral term gain */
      double Kd;  /*!< Differential term gain */
      
    } Coeff;
    
    std::array<double, 3> p;  /*!< Array including all PID parameter gains */
    
  }Parameters;
  

  
  /*
   * Controller saturation values
   */
  double out_min; /*!< Output min value */
  double out_max; /*!< Output max value */
  
  /*
   * Twiddle parameters
   */
  bool tw_flag;             /*!< Twiddle Flag activation */
  double tw_error;          /*!< Twiddle total error */
  bool increase;            /*!< Increase gain, flag */
  bool decrease;            /*!< Decrease gain, flag */
  int tw_step;              /*!< Twiddle step */
  double best_err;          /*!< Best error so far */
  std::array<double, 3> dp; /*!< Deltas of the parameters for adding or subtracting */
  int param_id;             /*!< Parameter index */
  
  const int tw_start = 100;   /*!< Start of evaluation for twiddle */
  const int tw_span = 6000;   /*!< Span of evaluation for twiddle */
  const double tw_tol = 0.2;  /*!< Twiddle tolerance */

  
  uWS::WebSocket<uWS::SERVER> ws; /*!< Server instance used to manipulate sim restart */
  
public:
  /**
  *   \brief Constructor
  *   \param Kp Value for proportional term gain
  *   \param Ki Value for integral term gain
  *   \param Kd Value for differential term gain
  *   \param min Value for minimum output of the controller
  *   \param max Value for maximum output of the controller
  *   \param use_twiddle Use twiddle algorithm flag
  *   \return Object of the PID class
  */
  PID(double Kp, double Ki, double Kd, double min, double max, bool use_twiddle);

  /**
  *   \brief Destructor.
  */
  virtual ~PID();

  /**
  *   \brief Update the PID error variables given cross track error.
  *   \param cte Cross Track Error
  */
  void UpdateError(double cte);

  /**
  *   \brief PID Controller main function
  *   \return Output value of the PID controller
  */
  double RunController();

  /**
  *   \brief Calculate the total PID error.
  *   \return The Total PID error
  */
  double TotalError();
  
  /**
   *  \brief Twiddle algorithm initializer
   */
  void InitTwiddle();
  
  /**
   *  \brief Twiddle algorithm main function
   */
  void Twiddle();
  
  /**
   *  \brief Restart Simulation server and rerun the experiment
   *  \param ws The websocket server of the simulation window
   */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
  
  /**
   *  \brief Set the simulation server to manipulate via the PID class
   *  \param ws The websocket server of the simulation window
   */
  void SetServer(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
