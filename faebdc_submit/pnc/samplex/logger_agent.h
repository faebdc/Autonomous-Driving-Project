// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent.h"
#include "homework6/simulation/vehicle_agent_factory.h"

#include "homework6/agents/Chenyao2333/planning.h"

namespace Chenyao2333 {

class LoggerAgent : public simulation::VehicleAgent {
 public:
  explicit LoggerAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {}

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
    interface::control::ControlCommand command;

    double now = agent_status.simulation_status().simulation_time();
    double velocity = CalcVelocity(agent_status.vehicle_status().velocity());
    auto position = agent_status.vehicle_status().position();

    velocity_reached_threshold_ = velocity > 5;
    velocity_reached_zero = velocity < 0.03;

    if (in_exp == false) {
      in_exp = true;
      exp_round++;
      if (exp_round > 30) {
        return command;
      }

      std::cerr << "Round: " << exp_round << std::endl;

      // the accelating experiment up!
      in_accelerate_exp = true;
      position_since_start = position;
      sec_since_start = now;
    }

    double brake = 0.2 + (0.3 - 0.2) / 30 * exp_round;
    double throttle = 0.2 + (0.4 - 0.2) / 30 * exp_round;

    // the accelating experiment done!
    if (velocity_reached_threshold_ && in_accelerate_exp) {
      in_accelerate_exp = false;
      double cost = now - sec_since_start;
      double dis = CalcDistance(position, position_since_start);
      log(THROTTLE, throttle, dis, cost);
      sec_since_reaching_threshold = now;
    }

    // wait for 3 secs, and check wheteher we haven't do the break experiment, then start it!
    if (sec_since_reaching_threshold > 0 && now - sec_since_reaching_threshold > 3 &&
        sec_since_break < 0 && !in_break_exp) {
      in_break_exp = true;
      sec_since_break = now;
      position_since_break = position;
    }

    // the break experiments done!
    if (in_break_exp && velocity_reached_zero) {
      in_break_exp = false;
      double cost = now - sec_since_break;
      double dis = CalcDistance(position, position_since_break);
      log(BRAKE, brake, dis, cost);
      sec_since_reaching_zero = now;
    }

    if (sec_since_break > 0) {
      command.set_brake_ratio(brake);
    } else if (!velocity_reached_threshold_ && sec_since_break < 0) {
      command.set_throttle_ratio(throttle);
    }

    // stop for a while, then next exp!
    if (now - sec_since_reaching_zero > 3 && sec_since_reaching_zero > 0) {
      in_exp = false;
      sec_since_start = -1;
      sec_since_break = -1;
      sec_since_reaching_threshold = -1;
      sec_since_reaching_zero = -1;
    }

    // log the velocitys
    /*if (std::abs(velocity) > 0.05) {
      std::ofstream output;
      output.open("/tmp/velocity_0.txt", std::ofstream::out | std::ofstream::app);
      output << agent_status.simulation_status().simulation_time() << " " << velocity << std::endl;
      output.close();
    }*/

    return command;
  }

 private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }

  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Vector3d& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    ;
    return std::sqrt(sqr_sum);
  }

  const int THROTTLE = 0, BRAKE = 1;

  void log(int type, double value, double dis, double cost) {
    std::string t;
    if (type == THROTTLE) {
      t = "THROTTLE";
    } else {
      t = "BRAKE";
    }
    std::ofstream output;
    output.open("/tmp/table.txt", std::ofstream::out | std::ofstream::app);
    output << t << " " << value << " " << dis << " " << cost << std::endl;
    output.close();
  }

  int exp_round = 0;

  bool velocity_reached_threshold_ = false;
  bool velocity_reached_zero = false;
  bool in_exp = false;

  double sec_since_start = -1;
  double sec_since_break = -1;

  double sec_since_reaching_threshold = -1;
  double sec_since_reaching_zero = -1;

  interface::geometry::Vector3d position_since_start, position_since_break;
  bool in_accelerate_exp = false, in_break_exp = false;
};

}  // namespace Chenyao2333
