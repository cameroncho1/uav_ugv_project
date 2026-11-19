#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <uav_ugv_msgs/evaluation_trace.hpp>
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
    const auto trace_topic = sdf->Get<std::string>("evaluation_topic", input_ + "/evaluation").first;
    if (!trace_topic.empty()) {
      trace_pub_ = transport_.Advertise<gz::msgs::StringMsg>(trace_topic);
    }
    if (!transport_.Subscribe(input_, &CommandWatchdog::on_command, this)) {
      throw std::runtime_error("Cannot subscribe to actuator command input");
    }
    worker_ = std::thread(
      [this]() {
        while (!stopping_) {
          gz::msgs::Twist output;
          int64_t input_sequence, age_ns;
          bool expired;
          {
            const std::lock_guard<std::mutex> lock(mutex_);
            age_ns = receipt_ns_ ? uav_ugv_msgs::trace_steady_ns() - receipt_ns_ : 0;
            const auto age = age_ns * 1e-9;
            input_sequence = accepted_sequence_;
            expired = !have_command_ || age > timeout_;
            if (have_command_ && age >= 0 && age <= timeout_) {output = command_;}
          }
          const bool queued = publisher_.Publish(output);
          const auto emitted = uav_ugv_msgs::trace_steady_ns();
          const auto output_sequence = output_sequence_++;
          if (trace_pub_ && trace_pub_.HasConnections()) {
            uav_ugv_msgs::EvaluationJson trace;
            trace.integer("schema_version", 1).text("kind", "watchdog_output")
              .text("host_id", uav_ugv_msgs::trace_host_id()).integer("sequence", output_sequence)
              .integer("steady_ns", emitted).integer("input_sequence", input_sequence)
              .integer("input_age_ns", age_ns).boolean("expired", expired).boolean("queued", queued)
              .number("linear", output.linear().x()).number("angular", output.angular().z());
            publish_trace(trace);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
  }

private:
  void on_command(const gz::msgs::Twist & command)
  {
    const auto & l = command.linear();
    const auto & a = command.angular();
    bool accepted = true;
    for (const auto v : {l.x(), l.y(), l.z(), a.x(), a.y(), a.z()}) {
      accepted = accepted && std::isfinite(v);
    }
    int64_t received, sequence;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      received = uav_ugv_msgs::trace_steady_ns();
      sequence = input_sequence_++;
      if (accepted) {
        command_ = command;
        receipt_ns_ = received;
        accepted_sequence_ = sequence;
        have_command_ = true;
      } else {
        have_command_ = false;
        command_.Clear();
      }
    }
    if (trace_pub_ && trace_pub_.HasConnections()) {
      uav_ugv_msgs::EvaluationJson trace;
      trace.integer("schema_version", 1).text("kind", "watchdog_input")
        .text("host_id", uav_ugv_msgs::trace_host_id()).integer("sequence", sequence)
        .integer("steady_ns", received).boolean("accepted", accepted)
        .number("linear", accepted ? l.x() : 0.0).number("angular", accepted ? a.z() : 0.0)
        .number("timeout_s", timeout_);
      publish_trace(trace);
    }
  }

  void publish_trace(const uav_ugv_msgs::EvaluationJson & trace)
  {
    gz::msgs::StringMsg message;
    message.set_data(trace.str());
    trace_pub_.Publish(message);
  }

  std::string input_;
  double timeout_{0.35};
  std::mutex mutex_;
  gz::msgs::Twist command_;
  int64_t receipt_ns_{0}, input_sequence_{0}, output_sequence_{0}, accepted_sequence_{-1};
  bool have_command_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  gz::transport::Node::Publisher publisher_;
  gz::transport::Node::Publisher trace_pub_;
  gz::transport::Node transport_;
};
}  // namespace ugv_control

GZ_ADD_PLUGIN(ugv_control::CommandWatchdog, gz::sim::System, gz::sim::ISystemConfigure)
