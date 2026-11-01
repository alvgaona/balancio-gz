// ROS 2 node: NMPC balancing + velocity/heading tracking for Balancio, the
// C++ counterpart of balancio_control/mpc_node.py. Same topics, same
// parameters (mpc.yaml, node name balancio_mpc); acados instead of CasADi's
// NLP solvers, so the Python-only "solver" parameter is ignored and "rti"
// picks SQP-RTI over full SQP.
//
// Subscribes
//   /balancio/imu      sensor_msgs/Imu  (the only sensor: no wheel encoders)
//   /cmd_vel           geometry_msgs/Twist  (linear.x [m/s], angular.z [rad/s])
// Publishes
//   /balancio/{left,right}_wheel/duty  std_msgs/Float64 [-1, 1]
//   /balancio/odom                     nav_msgs/Odometry (dead reckoning)
//   /balancio/mpc/predicted_path       nav_msgs/Path
//   /balancio/mpc/state, /reference    std_msgs/Float64MultiArray [x, th, psi, dx, dth, dpsi]
//   /balancio/mpc/input                std_msgs/Float64MultiArray [duty_L, duty_R, solve_ms, iters]
// First: rosidl_runtime_cpp/traits.hpp uses std::wstring_convert but only
// includes <codecvt>, and newer libc++ no longer pulls in <locale> with it.
#include <locale>
// clang-format off
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <vector>
// clang-format on

#include "balancio_nmpc/ekf.hpp"
#include "balancio_nmpc/nmpc.hpp"

using namespace std::chrono_literals;
using balancio_nmpc::Input;
using balancio_nmpc::NU;
using balancio_nmpc::NX;
using balancio_nmpc::State;
using Float64 = std_msgs::msg::Float64;
using Array = std_msgs::msg::Float64MultiArray;

namespace {

geometry_msgs::msg::Quaternion yaw_pitch_to_quat(double yaw, double pitch = 0.0,
                                                 double roll = 0.0) {
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  geometry_msgs::msg::Quaternion q;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}

template <std::size_t N>
std::array<double, N> to_array(const std::vector<double> &v, const char *name) {
  if (v.size() != N)
    throw std::invalid_argument(std::string(name) + ": expected " + std::to_string(N) +
                                " values");
  std::array<double, N> a;
  std::copy(v.begin(), v.end(), a.begin());
  return a;
}

Array array_msg(const double *v, std::size_t n) {
  Array m;
  m.data.assign(v, v + n);
  return m;
}

}  // namespace

class BalancioMpcNode : public rclcpp::Node {
 public:
  BalancioMpcNode() : Node("balancio_mpc") {
    const std::string default_model =
        ament_index_cpp::get_package_share_directory("balancio_description") +
        "/config/balancio.yaml";
    const balancio_nmpc::NmpcConfig d;
    const auto vec = [](const auto &a) { return std::vector<double>(a.begin(), a.end()); };

    const auto model_file = declare_parameter("model_file", default_model);
    balancio_nmpc::NmpcConfig c;
    c.dt = declare_parameter("dt", d.dt);
    c.horizon = static_cast<int>(declare_parameter("horizon", static_cast<int64_t>(d.horizon)));
    c.q = to_array<NX>(declare_parameter("q", vec(d.q)), "q");
    c.r = to_array<NU>(declare_parameter("r", vec(d.r)), "r");
    c.r_rate = to_array<NU>(declare_parameter("r_rate", vec(d.r_rate)), "r_rate");
    c.max_duty = declare_parameter("max_duty", d.max_duty);
    c.max_duty_rate = declare_parameter("max_duty_rate", d.max_duty_rate);
    c.max_theta = declare_parameter("max_theta", d.max_theta);
    c.rti = declare_parameter("rti", d.rti);
    fall_angle_ = declare_parameter("fall_angle", 1.0);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
    max_speed_ = declare_parameter("max_speed", 0.6);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 3.0);
    max_pos_lead_ = declare_parameter("max_position_lead", 0.2);
    max_head_lead_ = declare_parameter("max_heading_lead", 0.6);
    const auto ekf_q = to_array<NX>(
        declare_parameter("ekf_q", std::vector<double>{1e-4, 1e-3, 1e-3, 1e-3, 5e-2, 2e-2}),
        "ekf_q");
    const auto ekf_r = to_array<4>(
        declare_parameter("ekf_r", std::vector<double>{3e-3, 1e-2, 5e-3, 5e-3}), "ekf_r");
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    const bool publish_tf = declare_parameter("publish_tf", true);

    params_ = balancio_nmpc::Params::from_yaml(model_file);
    // The generated model (EKF, acados dynamics) has the parameters baked in
    // at build time; model_file only feeds what's computed here at startup.
    const auto t0 = std::chrono::steady_clock::now();
    mpc_ = std::make_unique<balancio_nmpc::Nmpc>(params_, c);
    const double build_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(get_logger(),
                "NMPC ready (acados %s, N=%d, dt=%gs, |duty|<=%g, stall %g Nm, no-load %g "
                "rad/s) built in %.2fs",
                c.rti ? "SQP-RTI" : "SQP", c.horizon, c.dt, c.max_duty, params_.max_torque,
                params_.motor_no_load_speed, build_s);
    ekf_ = std::make_unique<balancio_nmpc::ImuEkf>(c.dt, ekf_q, ekf_r);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "/balancio/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { on_imu(*m); });
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { on_cmd_vel(*m); });

    pub_left_ = create_publisher<Float64>("/balancio/left_wheel/duty", 10);
    pub_right_ = create_publisher<Float64>("/balancio/right_wheel/duty", 10);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/balancio/odom", 10);
    pub_pred_ = create_publisher<nav_msgs::msg::Path>("/balancio/mpc/predicted_path", 10);
    pub_state_ = create_publisher<Array>("/balancio/mpc/state", 10);
    pub_ref_ = create_publisher<Array>("/balancio/mpc/reference", 10);
    pub_input_ = create_publisher<Array>("/balancio/mpc/input", 10);
    if (publish_tf) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timers on the node's clock (sim time under Gazebo), as rclpy's create_timer.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(c.dt),
                                  [this] { step(); });
    report_timer_ = rclcpp::create_timer(this, get_clock(), 5s, [this] { report(); });
  }

  void stop() { publish_duty({0.0, 0.0}); }

 private:
  void on_imu(const sensor_msgs::msg::Imu &m) {
    const auto &o = m.orientation;
    const auto &w = m.angular_velocity;
    ekf_->update_imu({o.x, o.y, o.z, o.w}, {w.x, w.y, w.z}, rclcpp::Time(m.header.stamp).seconds());
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist &m) {
    v_cmd_ = std::clamp(m.linear.x, -max_speed_, max_speed_);
    w_cmd_ = std::clamp(m.angular.z, -max_yaw_rate_, max_yaw_rate_);
    last_cmd_time_ = now();
  }

  void publish_duty(const Input &u) {
    Float64 l, r;
    l.data = u[0];
    r.data = u[1];
    pub_left_->publish(l);
    pub_right_->publish(r);
    ekf_->set_input(now().seconds(), u);
  }

  void step() {
    if (!ekf_->ready()) return;
    const rclcpp::Time t_now = now();
    const State x = ekf_->step(t_now.seconds());
    const auto &c = mpc_->config();
    const double dt = c.dt;

    if (!last_cmd_time_ || (t_now - *last_cmd_time_).seconds() > cmd_timeout_)
      v_cmd_ = w_cmd_ = 0.0;
    if (!x_ref_) {
      x_ref_ = x[0];
      psi_ref_ = x[2];
    }

    if (std::fabs(x[1]) > fall_angle_) {
      if (!fallen_)
        RCLCPP_WARN(get_logger(), "Robot fell (pitch %.1f deg). Motors off.", x[1] * 180 / M_PI);
      fallen_ = true;
      u_prev_ = {0.0, 0.0};
      mpc_->reset();
      ekf_->reset();
      x_ref_ = x[0];
      psi_ref_ = x[2];
      publish_duty(u_prev_);
      return;
    }
    if (fallen_ && std::fabs(x[1]) < 0.2) {
      RCLCPP_INFO(get_logger(), "Robot upright again, NMPC re-enabled.");
      fallen_ = false;
    }

    // Advance the reference, never letting it run away from the robot.
    x_ref_ = std::clamp(*x_ref_ + v_cmd_ * dt, x[0] - max_pos_lead_, x[0] + max_pos_lead_);
    psi_ref_ = std::clamp(psi_ref_ + w_cmd_ * dt, x[2] - max_head_lead_, x[2] + max_head_lead_);
    std::vector<State> xref(c.horizon + 1);
    for (int k = 0; k <= c.horizon; ++k)
      xref[k] = {*x_ref_ + v_cmd_ * k * dt, 0.0, psi_ref_ + w_cmd_ * k * dt, v_cmd_, 0.0, w_cmd_};

    const auto t0 = std::chrono::steady_clock::now();
    const auto sol = mpc_->solve(x, xref, u_prev_);
    const double solve_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    solve_times_.push_back(solve_ms);
    if (!sol.success())
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "NMPC solver: %d (%d it)", sol.status,
                           sol.iterations);
    u_prev_ = sol.u;
    publish_duty(sol.u);
    publish_debug(x, xref[0], sol, solve_ms, t_now);
  }

  void publish_debug(const State &x, const State &xref0, const balancio_nmpc::NmpcSolution &sol,
                     double solve_ms, const rclcpp::Time &t_now) {
    pub_state_->publish(array_msg(x.data(), NX));
    pub_ref_->publish(array_msg(xref0.data(), NX));
    const double in[4] = {sol.u[0], sol.u[1], solve_ms, static_cast<double>(sol.iterations)};
    pub_input_->publish(array_msg(in, 4));

    const double X = ekf_->X(), Y = ekf_->Y();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t_now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = params_.r;
    odom.pose.pose.orientation = yaw_pitch_to_quat(x[2], x[1], ekf_->roll());
    odom.twist.twist.linear.x = x[3];
    odom.twist.twist.angular.y = x[4];
    odom.twist.twist.angular.z = x[5];
    pub_odom_->publish(odom);

    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = odom.header;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = X;
      t.transform.translation.y = Y;
      t.transform.translation.z = params_.r;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(t);
    }

    // Predicted path, projected on the plane from the current pose.
    nav_msgs::msg::Path path;
    path.header = odom.header;
    double px = X, py = Y;
    for (std::size_t i = 0; i < sol.x.size(); ++i) {
      if (i > 0) {
        const double ds = sol.x[i][0] - sol.x[i - 1][0];
        px += ds * std::cos(sol.x[i][2]);
        py += ds * std::sin(sol.x[i][2]);
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = px;
      ps.pose.position.y = py;
      ps.pose.orientation = yaw_pitch_to_quat(sol.x[i][2]);
      path.poses.push_back(ps);
    }
    pub_pred_->publish(path);
  }

  void report() {
    if (solve_times_.empty()) return;
    double sum = 0.0, mx = 0.0;
    for (double s : solve_times_) {
      sum += s;
      mx = std::max(mx, s);
    }
    RCLCPP_INFO(get_logger(),
                "solve time: mean %.2f ms, max %.2f ms (%zu solves) | v_cmd=%.2f w_cmd=%.2f",
                sum / solve_times_.size(), mx, solve_times_.size(), v_cmd_, w_cmd_);
    solve_times_.clear();
  }

  balancio_nmpc::Params params_;
  std::unique_ptr<balancio_nmpc::Nmpc> mpc_;
  std::unique_ptr<balancio_nmpc::ImuEkf> ekf_;

  double fall_angle_, cmd_timeout_, max_speed_, max_yaw_rate_, max_pos_lead_, max_head_lead_;
  std::string odom_frame_, base_frame_;

  Input u_prev_{};
  double v_cmd_ = 0.0, w_cmd_ = 0.0;
  std::optional<rclcpp::Time> last_cmd_time_;
  std::optional<double> x_ref_;
  double psi_ref_ = 0.0;
  bool fallen_ = false;
  std::vector<double> solve_times_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr pub_left_, pub_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pred_;
  rclcpp::Publisher<Array>::SharedPtr pub_state_, pub_ref_, pub_input_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_, report_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BalancioMpcNode>();
  rclcpp::spin(node);
  // spin returns on Ctrl-C, after rclcpp's context is shut down, so this
  // last zero duty mostly doesn't go out; same as the Python node.
  if (rclcpp::ok()) node->stop();
  rclcpp::shutdown();
  return 0;
}
