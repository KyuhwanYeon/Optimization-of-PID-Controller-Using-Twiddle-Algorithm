#ifndef PID_H
#define PID_H
#include "vector"
#include "iostream"
#include <limits>
#include <cmath>
using std::vector;
using namespace std;

class PID
{
public:
  // Constructor
  PID();
  // Deconstructor
  virtual ~PID();
  // Initializer
  void init(double Kp_, double Ki_, double Kd_, double dt, double min_out, double max_out);
  // Update error
  void update_error(double error);
  double total_error();
  // Mutators
  void set_gain(vector<double> gain);
  // Accessors
  vector<double> get_gain(void);

private:
  double p_error;
  double i_error;
  double d_error;
  double Kp;
  double Ki;
  double Kd;
  double pre_error;
  double integral_error;
  double dt;
  double min_out;
  double max_out;
};


class GainOptimizer
{
  public:
  // Constructor
  GainOptimizer(double gain_threshold, double time2stable, vector<double> cur_gain);
  // Accessors
  vector<double> get_gain(void);
  // Optimizer with twiddle
  void run_twiddle(double error);
  void print_gain(int num_gain);
  private:
  double time2stable;
  double gain_threshold;
  double best_err_sum;
  vector<double> gain;  // gain[0] = Kp, gain[1] = Ki, gain[2] = Kd
  vector<double> delta_gain;  //  of gainz
  bool flag_add;
  bool flag_subtract;
  int run_cnt;
  int num_gain;  // 0: Kp, 1: Ki, 2: Kd
};

#endif // PID_H