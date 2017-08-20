#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)  {
  //CEL: initialize the coefficients and errors
  PID::Kp = Kp;//proportional, steer back to centerline
  PID::Ki = Ki;//integral, handle offset
  PID::Kd = Kd;//differential, handle steering change rate degree

  p_error = 0.0; 
  i_error = 0.0; 
  d_error = 0.0; 
}

void PID::UpdateError(double cte) {
  if (p_error == 0.0)
    p_error = cte; //set the first p_error to the passed in CTE value
  d_error = cte - p_error;  //find difference between current and prior CTE
  p_error = cte;  //save prior CTE in next p_error
  i_error += cte; //sum all CTEs
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;  //calculate error, from the Q&A video
}
