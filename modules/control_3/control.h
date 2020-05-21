
#ifndef MODULES_CONTROL_CONTROL_H_
#define MODULES_CONTROL_CONTROL_H_

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/jmcauto_app.h"
#include "modules/common/util/util.h"
#include "modules/control/controller/controller_agent.h"

/**
 * @namespace jmcauto::control
 * @brief jmcauto::control
 */
namespace jmcauto {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chasiss, and
 * pad data to compute throttle, brake and steer values.
 */
class Control : public jmcauto::common::JmcAutoApp {
  friend class ControlTestBase;

 public:
  Control()
      : monitor_logger_(jmcauto::common::monitor::MonitorMessageItem::CONTROL) {}

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  jmcauto::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  jmcauto::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Control() = default;

 private:
  // Upon receiving pad message
  // void OnPad(const jmcauto::control::PadMessage &pad);
  // Upon receiving monitor message
  void OnMonitor(
      const jmcauto::common::monitor::MonitorMessage &monitor_message);
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);
  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput();
  common::Status CheckTimestamp();
  //common::Status CheckPad();
  void SendCmd(ControlCommand *control_command);
 private:
  double init_time_ = 0.0;
  localization::LocalizationEstimate localization_;
  canbus::Chassis chassis_;
  planning::ADCTrajectory trajectory_;
  //PadMessage pad_msg_;
  ControllerAgent controller_agent_;
  ControlConf control_conf_;
  jmcauto::common::monitor::MonitorLogger monitor_logger_; 
  bool estop_ = false;
  // bool pad_received_ = false;
  unsigned int status_lost_ = 0;//状态丢失
  unsigned int status_sanity_check_failed_ = 0;//状态完整性检查失败
  unsigned int total_status_lost_ = 0;//??
  unsigned int total_status_sanity_check_failed_ = 0;//
  
  ros::Timer timer_;
};

}  // namespace control
}  // namespace jmcauto

#endif  // MODULES_CONTROL_CONTROL_H_
