#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "santa_sim_interface/msg/sleigh_state.hpp"
#include "santa_sim_interface/srv/control_sleigh.hpp"

using SleighState = santa_sim_interface::msg::SleighState;
using ControlSleigh = santa_sim_interface::srv::ControlSleigh;

class SantaSimNode : public rclcpp::Node
{
  public:
    SantaSimNode(int timestep_ms, float k)
    : Node("santa_sim_node"), dt(timestep_ms / 1000.), k(k) {
      // Opret publisher, som kan publicere kane staten
      publisher_ = this->create_publisher<SleighState>(
        "santa_sim/sleigh_state", 10
      );

      // Opret service serveren, der kan styre kanen
      server_ = this->create_service<ControlSleigh>(
        "santa_sim/control_sleigh", 
        std::bind(&SantaSimNode::control_sleigh, this, 
                  std::placeholders::_1, std::placeholders::_2)
      );

      // Opret en timer, som stepper simuleringen 
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timestep_ms), 
        std::bind(&SantaSimNode::step, this)
      );

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Santa sim is running.");
    }
    
  private:
    float dt; // timestep, seconds
    float k; // damping constant
    float pos_x = 0., pos_y = 0., speed_x = 0., speed_y = 0.; // state
    
    rclcpp::Publisher<SleighState>::SharedPtr publisher_;
    rclcpp::Service<ControlSleigh>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;

    void control_sleigh(
        const std::shared_ptr<ControlSleigh::Request> request,
        std::shared_ptr<ControlSleigh::Response> response
    ){
        speed_x += request->delta_speed_x;
        speed_y += request->delta_speed_y;
        response->success = true;
        
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), 
            "Controlling sleigh:\ndelta_speed_x: %g, delta_speed_y: %g",
            request->delta_speed_x, 
            request->delta_speed_y
        );
    }

    void step(){
        // simple simulation with damping
        speed_x += -k * speed_x * dt;
        speed_y += -k * speed_y * dt;
    
        pos_x += speed_x * dt;
        pos_y += speed_y * dt;

        // publish state
        auto message = SleighState();
        message.pos_x = pos_x;
        message.pos_y = pos_y;
        message.speed_x = speed_x;
        message.speed_y = speed_y;
        publisher_->publish(message);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  int dt_ms = 20; // 50 hz
  float k = 0.1; // damping constant

  rclcpp::spin(std::make_shared<SantaSimNode>(dt_ms, k));
  rclcpp::shutdown();
}