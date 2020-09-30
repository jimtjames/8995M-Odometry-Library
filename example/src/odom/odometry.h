#include <vector>
#include <tuple>
class chassis {
    private:
      int8_t left_1, left_2, right_1, right_2;
      int8_t left_enc, right_enc, side_enc;
      int8_t inertial;
      double wheel_radius;
      double s_l, s_r, s_s;
      double x, y, theta;
      double x_prev, y_prev, theta_reset, theta_prev;
      double kP_u, kI_u, kD_u, kP_v, kI_v, kD_v, kP_t, kI_t, kD_t;
      int16_t voltage_max;
      int8_t lookahead_distance;
    public:
      bool moving;
      chassis(int8_t l1, int8_t l2, int8_t r1, int8_t r2,
      int8_t l, int8_t r, int8_t s, int8_t imu, double w_r, double sl,
      double sr, double ss, double x_init, double y_init, double theta_init,
      double kPu, double kIu, double kDu, double kPv, double kIv, double kDv,
      double kPt, double kIt, double kDt);
      std::tuple<double, double, double> get_position();
      void update_odometry();
      void drive_to(double x, double y);
      void drive_to_point(double x, double y, bool backwards = false);
      void move_forward(double inches);
      void turn_to(double degrees);
      void tune_drive_fwd();
      void follow_path(std::vector<std::tuple<double, double>>);
      double angle_to(double x_t, double y_t);
      void set_voltage_max(int16_t max);
      void set_position(double x, double y, double theta);
      void set_lookahead_distance(uint8_t dist);
      void drive_to_point_async(double x_t, double y_t, bool backwards);
      void wait_until_point(double x_t, double y_t);
};
struct Param {
  chassis* base;
  double x;
  double y;
  bool backwards;
};
void drive_to_point_helper(void* params);
void start_odometry(void*);
