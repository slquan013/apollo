/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/control/control.h"
#include <iomanip>
#include <string>
#include "ros/include/std_msgs/String.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

#include "modules/localization/proto/localization.pb.h"

namespace jmcauto {
namespace control {

using jmcauto::canbus::Chassis;
using jmcauto::common::ErrorCode;
using jmcauto::common::Status;
using jmcauto::common::VehicleStateProvider;
using jmcauto::common::adapter::AdapterManager;
using jmcauto::common::monitor::MonitorMessageItem;
using jmcauto::common::time::Clock;
using jmcauto::localization::LocalizationEstimate;
using jmcauto::planning::ADCTrajectory;

std::string Control::Name() const { return FLAGS_control_node_name; }

Status Control::Init() {
  init_time_ = Clock::NowInSeconds();//当前时间，单位秒
  AINFO << "Control init, starting ...";
  CHECK(common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;  //读控制配置文件lincoln.pb.txt
  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";
  AdapterManager::Init(FLAGS_control_adapter_config_filename);//读message消息类别，adapter.conf
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_); //
  // set controller
  if (!controller_agent_.Init(&control_conf_).ok()) {
    std::string error_msg = "Control init controller failed! Stopping...";
    buffer.ERROR(error_msg);
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }//注册控制器，目前只支持LON/LAN/MPC

  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  CHECK(AdapterManager::GetLocalization())<< "Localization is not initialized.";
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  CHECK(AdapterManager::GetPlanning()) << "Planning is not initialized.";
  //CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";
  CHECK(AdapterManager::GetMonitor()) << "Monitor is not initialized.";
  CHECK(AdapterManager::GetControlCommand())<< "ControlCommand publisher is not initialized.";
 // AdapterManager::AddPadCallback(&Control::OnPad, this);
  AdapterManager::AddMonitorCallback(&Control::OnMonitor, this);
  return Status::OK();
}

Status Control::Start() {
  // set initial vehicle state by cmd通过cmd初始化车辆状态
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so至少需要休眠80ms
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // should init_vehicle first, let car enter work status, then use status msg trigger control触发控制
  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
 // pad_msg_.set_action(control_conf_.action());
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(control_conf_.control_period()), &Control::OnTimer, this);
  AINFO << "Control init done!";
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("control started");
  return Status::OK();
}
/*
void Control::OnPad(const PadMessage &pad) {
  pad_msg_ = pad;
  ADEBUG << "Received Pad Msg:" << pad.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
  // do something according to pad message
  if (pad_msg_.action() == DrivingAction::RESET) {
    AINFO << "Control received RESET action!";
    estop_ = false;
  }
  pad_received_ = true;
}*/
void Control::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

void Control::OnTimer(const ros::TimerEvent &) {
  double start_timestamp = Clock::NowInSeconds();
  if (FLAGS_is_control_test_mode && FLAGS_control_test_duration > 0 &&
      (start_timestamp - init_time_) > FLAGS_control_test_duration) {
    AERROR << "Control finished testing. exit";
    ros::shutdown();
  }
  ControlCommand control_command;
  Status status = ProduceControlCommand(&control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  double end_timestamp = Clock::NowInSeconds();
  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms < control_conf_.control_period());
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());
  SendCmd(&control_command); 
}

Status Control::ProduceControlCommand(ControlCommand *control_command) {
  Status status = CheckInput();
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();//ERROR消息发布频率100hz
    control_command->mutable_engage_advice()->set_advice(
        jmcauto::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());   //???
    estop_ = true;
  } else {
    Status status_ts = CheckTimestamp();
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      estop_ = true;
      status = status_ts;
      if (chassis_.driving_mode() !=
          jmcauto::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            jmcauto::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          jmcauto::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }
  estop_ = estop_ || trajectory_.estop().is_estop();
  if (!estop_) {
    if (chassis_.driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();//
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";//手动模式下重置控制器
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(localization_.header());
    debug->mutable_canbus_header()->CopyFrom(chassis_.header());
    debug->mutable_trajectory_header()->CopyFrom(trajectory_.header());

    Status status_compute = controller_agent_.ComputeControlCommand(
        &localization_, &chassis_, &trajectory_, control_command);

    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: " << localization_.ShortDebugString()
             << " with chassis: " << chassis_.ShortDebugString()
             << " with trajectory: " << trajectory_.ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      status = status_compute;
    }
  }
  //急停模式
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
  // check signal
  if (trajectory_.decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        trajectory_.decision().vehicle_signal());
  }
  return status;
}


Status Control::CheckInput() {
  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()) {
    AWARN_EVERY(100) << "No Localization msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No localization msg");
  }
  localization_ = localization_adapter->GetLatestObserved();//返回观察队列最新数据，调用之前需调用Empty()确定是否有数据
  ADEBUG << "Received localization:" << localization_.ShortDebugString();//定位数据

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()) {
    AWARN_EVERY(100) << "No Chassis msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No chassis msg");
  }
  chassis_ = chassis_adapter->GetLatestObserved();//底盘数据
  ADEBUG << "Received chassis:" << chassis_.ShortDebugString();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  if (trajectory_adapter->Empty()) {
    AWARN_EVERY(100) << "No planning msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No planning msg");
  }
  trajectory_ = trajectory_adapter->GetLatestObserved();//planning轨迹点
  if (!trajectory_.estop().is_estop() &&
      trajectory_.trajectory_point_size() == 0) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point.");
  }

  for (auto &trajectory_point : *trajectory_.mutable_trajectory_point()) {
    if (trajectory_point.v() < control_conf_.minimum_speed_resolution()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  VehicleStateProvider::instance()->Update(localization_, chassis_);

  return Status::OK();
}

Status Control::CheckTimestamp() {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - localization_.header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff = current_timestamp - chassis_.header().timestamp_sec();
  if (chassis_diff >(FLAGS_max_chassis_miss_num * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }
  double trajectory_diff =
      current_timestamp - trajectory_.header().timestamp_sec();
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

void Control::SendCmd(ControlCommand *control_command) {
  // set header
  AdapterManager::FillControlCommandHeader(Name(), control_command);
  AdapterManager::PublishControlCommand(*control_command);
}

void Control::Stop() {}

}  // namespace control
}  // namespace jmcauto
