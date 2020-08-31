#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::init(double Kp_, double Ki_, double Kd_, double dt, double min_out, double max_out)
{
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->pre_error = 0;
  this->integral_error = 0;
  this->dt = dt;
  this->min_out = min_out;
  this->max_out = max_out;
}

void PID::update_error(double error)
{
  integral_error += error * dt;
  double derivative = (error - pre_error) / dt;
  pre_error = error;
  p_error = Kp * error;
  i_error = Ki * integral_error;
  d_error = Kd * derivative;
}

double PID::total_error()
{
  double error_sum = p_error + i_error + d_error;
  error_sum = error_sum > max_out ? max_out : error_sum; // min, max threshold
  error_sum = error_sum < min_out ? min_out : error_sum; // min, max threshold
  return error_sum;                                      
}
void PID::set_gain(vector<double> gain)
{
  this->Kp = gain[0];
  this->Ki = gain[1];
  this->Kd = gain[2];
}
vector<double> PID::get_gain(void)
{
  return {Kp, Ki, Kd};
}

GainOptimizer::GainOptimizer(double gain_threshold, double time2stable, vector<double> cur_gain) : gain_threshold(gain_threshold), time2stable(time2stable)
{
  run_cnt = 0;
  num_gain = 0;
  best_err_sum = 9999;
  for (int i = 0; i < cur_gain.size(); i++)
  {
    this->gain.push_back(cur_gain[i]);
    this->delta_gain.push_back(0.1 * cur_gain[i]);
  }
  flag_add = flag_subtract = false;
}
vector<double> GainOptimizer::get_gain(void)
{
  return gain;
}
void GainOptimizer::run_twiddle(double error)
{
  double sum_d_gain = delta_gain[0] + delta_gain[1] + delta_gain[2];
  static double cur_err_sum = 0;
  if (sum_d_gain > gain_threshold)
  {

    if (run_cnt < time2stable)
    {
      cur_err_sum += error * error;
      run_cnt++;
      return;
    }
    else
    {
      run_cnt = 0;
      // Check current error with best error
      printf("\ncurrent err: %lf, best err: %lf, sum of delta gain: %lf \n", cur_err_sum, best_err_sum, sum_d_gain);
      if (cur_err_sum < best_err_sum)
      {
        printf("Improved!\n");
        best_err_sum = cur_err_sum;
        // go to next gain
        num_gain = (num_gain + 1) % 3;
        flag_add = flag_subtract = false;
      }
      // Update gain
      if (!flag_add && !flag_subtract)
      {
        gain[num_gain] += 1.1 * delta_gain[num_gain];
        flag_add = true;
        print_gain(num_gain);
      }
      else if (flag_add && !flag_subtract)
      {
        gain[num_gain] -= 2 * delta_gain[num_gain];
        flag_subtract = true;
        print_gain(num_gain);
      }
      else
      {
        gain[num_gain] += delta_gain[num_gain];
        delta_gain[num_gain] *= 0.9;
        print_gain(num_gain);
        // go to next gain
        num_gain = (num_gain + 1) % 3;
        flag_add = flag_subtract = false;
      }
      cur_err_sum = 0;
    }
  }
  else
  {
    printf("Optimized done! \n");
    return;
  }
}

void GainOptimizer::print_gain(int num_gain)
{
  if (!flag_add && !flag_subtract)
  {
    if (num_gain == 0)
    {
      printf("Add gain at P\n");
    }
    else if (num_gain == 1)
    {
      printf("Add gain at I\n");
    }
    else
    {
      printf("Add gain at D\n");
    }
    printf("P: %lf, I: %lf, D: %lf \n", gain[0], gain[1], gain[2]);
  }
  else if (flag_add && !flag_subtract)
  {
    if (num_gain == 0)
    {
      printf("Subtract gain at P\n");
    }
    else if (num_gain == 1)
    {
      printf("Subtract gain at I\n");
    }
    else
    {
      printf("Subtract gain at D\n");
    }
    printf("P: %lf, I: %lf, D: %lf \n", gain[0], gain[1], gain[2]);
  }
  else
  {
    if (num_gain == 0)
    {
      printf("Set back gain at P\n");
    }
    else if (num_gain == 1)
    {
      printf("Set back gain at I\n");
    }
    else
    {
      printf("Set back gain at D\n");
    }
    printf("P: %lf, I: %lf, D: %lf \n", gain[0], gain[1], gain[2]);
  }
}
