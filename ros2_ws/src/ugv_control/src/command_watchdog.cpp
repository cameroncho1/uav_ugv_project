#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace ugv_control
{
// This driver boundary runs inside the actuator's Gazebo process. It requires
// neither the ROS controller nor the ROS bridge, and uses a steady clock.
class CommandWatchdog : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  ~CommandWatchdog() override
  {
    transport_.Unsubscribe(input_);
    stopping_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    input_ = sdf->Get<std::string>("input_topic");
    const auto output = sdf->Get<std::string>("output_topic");
    timeout_ = sdf->Get<double>("timeout");
    if (input_.empty() || output.empty() || input_ == output ||
      !std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 5)
    {
      throw std::invalid_argument("Invalid actuator command watchdog configuration");
    }
    publisher_ = transport_.Advertise<gz::msgs::Twist>(output);
    if (!transport_.Subscribe(input_, &CommandWatchdog::on_command, this)) {
      throw std::runtime_error("Cannot subscribe to actuator command input");
    }
    worker_ = std::thread(
      [this]() {
        while (!stopping_) {
          gz::msgs::Twist output;
          {
            const std::lock_guard<std::mutex> lock(mutex_);
            const auto age = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - receipt_).count();
            if (have_command_ && age >= 0 && age <= timeout_) {output = command_;}
          }
          publisher_.Publish(output);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
  }

private:
  void on_command(const gz::msgs::Twist & command)
  {
    const auto & l = command.linear();
    const auto & a = command.angular();
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const auto v : {l.x(), l.y(), l.z(), a.x(), a.y(), a.z()}) {
      if (!std::isfinite(v)) {have_command_ = false; command_.Clear(); return;}
    }
    command_ = command;
    receipt_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  std::string input_;
  double timeout_{0.35};
  std::mutex mutex_;
  gz::msgs::Twist command_;
  std::chrono::steady_clock::time_point receipt_;
  bool have_command_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  gz::transport::Node::Publisher publisher_;
  gz::transport::Node transport_;
};
}  // namespace ugv_control

GZ_ADD_PLUGIN(ugv_control::CommandWatchdog, gz::sim::System, gz::sim::ISystemConfigure)
