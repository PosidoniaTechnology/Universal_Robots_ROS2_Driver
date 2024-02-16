// Copyright 2019, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "scaled_joint_trajectory_controller_parameters.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ur_controllers
{
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;



  void set_hold_position() override
  {
    scaled_param_listener_->refresh_dynamic_parameters();
    scaled_params_ = scaled_param_listener_->get_params();

    bool stopped = false;

    stopping_scaling_factor_ = scaling_factor_;
    use_stopping_ = true;

    while (!stopped && rclcpp::ok() && (traj_point_active_ptr_ != nullptr)) {
      stopped = std::all_of(state_current_.velocities.begin(), state_current_.velocities.end(),
                            [=](const double v) { return std::abs(v) < 1e-6; });

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    trajectory_msgs::msg::JointTrajectory empty_msg;
    empty_msg.header.stamp = rclcpp::Time(0);

    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg);
    add_new_trajectory_msg(traj_msg);

    use_stopping_ = false;
  }

protected:
  struct TimeData
  {
    TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0)), uptime(0.0)
    {
    }
    rclcpp::Time time;
    rclcpp::Duration period;
    rclcpp::Time uptime;
  };

    rclcpp_action::GoalResponse goal_received_callback(
            const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

        if (use_stopping_){
            RCLCPP_ERROR(get_node()->get_logger(), "Stopping active, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Precondition: Running controller
        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_ERROR(
                    get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!validate_trajectory_msg(goal->trajectory))
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse goal_cancelled_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

        // Check that cancel request refers to currently active goal (if any)
        const auto active_goal = *rt_active_goal_.readFromNonRT();
        if (active_goal && active_goal->gh_ == goal_handle)
        {
            // Controller uptime
            // Enter hold current position mode
            // can last longer so it can actually succeed in between
            set_hold_position();

            RCLCPP_DEBUG(
                    get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

            // check if it is still active
            const auto active_goal = *rt_active_goal_.readFromNonRT();
            if (active_goal && active_goal->gh_ == goal_handle) {

                // Mark the current goal as canceled
                auto action_res = std::make_shared<FollowJTrajAction::Result>();
                active_goal->setCanceled(action_res);
                rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            }else{
                return rclcpp_action::CancelResponse::REJECT;
            }
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

private:
  double scaling_factor_{};
  double stopping_scaling_factor_{};
  std::atomic<bool> use_stopping_ = false;
  realtime_tools::RealtimeBuffer<TimeData> time_data_;

  std::shared_ptr<scaled_joint_trajectory_controller::ParamListener> scaled_param_listener_;
  scaled_joint_trajectory_controller::Params scaled_params_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
