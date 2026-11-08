// New driver implementation; legacy wire contract documented in NOTICE.md.
// No allocation, DOB, payload sequence or PX4 ServoAngle logic is implemented here.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "palletrone_task_flight/joint_math.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using Steady=std::chrono::steady_clock;
using namespace std::chrono_literals;

class Driver : public rclcpp::Node {
 public:
  Driver() : Node("dynamixel_fix_with_j1") {
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("hardware driver requires system time");
    rcl_interfaces::msg::ParameterDescriptor ro; ro.read_only=true;
    dry_=declare_parameter("dry_run",true,ro);
    device_=declare_parameter<std::string>("device","/dev/ttyU2D2",ro);
    baud_=declare_parameter("baud_rate",1000000,ro);
    id_=declare_parameter("j1_id",5,ro);
    cal_.zero_tick=declare_parameter("zero_tick",2048,ro);
    cal_.direction=declare_parameter("direction",1,ro);
    cal_.ticks_per_rev=declare_parameter("ticks_per_rev",4096,ro);
    cal_.raw_min=declare_parameter("raw_tick_min",0,ro);
    cal_.raw_max=declare_parameter("raw_tick_max",4095,ro);
    cal_.angle_min=declare_parameter("physical_min_deg",-5.0,ro)*task_flight::pi/180;
    cal_.angle_max=declare_parameter("physical_max_deg",35.0,ro)*task_flight::pi/180;
    cal_.confirmed=declare_parameter("calibration_confirmed",false,ro);
    table_confirmed_=declare_parameter("control_table_confirmed",false,ro);
    model_=declare_parameter("expected_j1_model_number",0,ro);
    rate_=declare_parameter("max_joint_rate_deg_s",15.0,ro)*task_flight::pi/180;
    joint_period_=declare_parameter("joint_period_s",0.05,ro);
    joint_timeout_ms_=declare_parameter("joint_rx_timeout_ms",2.0,ro);
    legacy_timeout_ms_=declare_parameter("legacy_rx_timeout_ms",4.0,ro);
    command_timeout_=declare_parameter("command_timeout_s",0.5,ro);
    cal_.validate();
    if (id_<5 || id_>252 || baud_<=0 || model_<0 || model_>65535
        || !std::isfinite(rate_) || rate_<=0 || rate_>task_flight::pi
        || !std::isfinite(joint_period_) || joint_period_<0.05
        || !std::isfinite(joint_timeout_ms_) || joint_timeout_ms_<=0 || joint_timeout_ms_>5
        || !std::isfinite(legacy_timeout_ms_) || legacy_timeout_ms_<=0 || legacy_timeout_ms_>5
        || !std::isfinite(command_timeout_) || command_timeout_<=0)
      throw std::invalid_argument("invalid driver settings");
    if (!dry_ && (!cal_.confirmed || !table_confirmed_ || model_==0))
      throw std::invalid_argument("live driver requires calibration, control table and model confirmation");
    auto sensor=rclcpp::SensorDataQoS().keep_last(1);
    feedback_=create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angle",10);
    angle_=create_publisher<std_msgs::msg::Float32>("/j1_angle",sensor);
    state_=create_publisher<sensor_msgs::msg::JointState>("/j1_state",sensor);
    ready_=create_publisher<std_msgs::msg::Bool>("/j1_ready",sensor);
    target_=create_publisher<std_msgs::msg::Float32>("/j1_angle_target",sensor);
    limits_=create_publisher<std_msgs::msg::Float64MultiArray>("/j1_limits",sensor);
    legacy_sub_=create_subscription<std_msgs::msg::Float32MultiArray>("/servo_angle_cmd",sensor,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        try {legacy_goal_=task_flight::legacy_commands(msg->data); legacy_command_at_=Steady::now(); have_legacy_command_=true;}
        catch(const std::exception& e) {RCLCPP_ERROR(get_logger(),"%s",e.what());}
      });
    joint_sub_=create_subscription<std_msgs::msg::Float32>("/j1_angle_cmd",sensor,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        if (dry_ || !opened_ || joint_failed_) return;
        try {cal_.raw(msg->data); requested_theta_=msg->data; joint_command_at_=Steady::now(); have_joint_command_=true;}
        catch(const std::exception& e) {joint_fault(e.what());}
      });
    born_=Steady::now();
    timer_=create_wall_timer(10ms,[this] {tick();});
    RCLCPP_INFO(get_logger(),"dry_run=%s; payload slot remains legacy; J1 is independent",dry_?"true":"false");
  }

  ~Driver() override {
    // Never disable ID1..4 torque as a side effect of a J1 fault or process exit.
    if (exclusive_fd_>=0) {ioctl(exclusive_fd_,TIOCNXCL); close(exclusive_fd_);}
    if (port_) port_->closePort();
    if (lock_fd_>=0) close(lock_fd_);
  }

 private:
  bool dry_=true, opened_=false, startup_failed_=false, joint_failed_=false, table_confirmed_=false;
  bool have_legacy_command_=false, have_joint_command_=false, legacy_conflict_=false;
  int baud_=0,id_=5,model_=0,lock_fd_=-1,exclusive_fd_=-1;
  std::string device_;
  task_flight::Calibration cal_;
  double rate_=0, joint_period_=0.05, joint_timeout_ms_=2,legacy_timeout_ms_=4,command_timeout_=0.5;
  double requested_theta_=0, commanded_theta_=0;
  Steady::time_point born_,last_joint_{},last_graph_{},legacy_command_at_{},joint_command_at_{};
  std::array<uint32_t,4> legacy_goal_{};
  std::unique_ptr<dynamixel::PortHandler> port_;
  dynamixel::PacketHandler* packet_=nullptr;
  std::unique_ptr<dynamixel::GroupSyncRead> read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> write_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr feedback_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_,target_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr limits_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr legacy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_sub_;

  static double age(Steady::time_point a,Steady::time_point b) {return std::chrono::duration<double>(a-b).count();}
  void ready(bool value) {std_msgs::msg::Bool msg;msg.data=value;ready_->publish(msg);}
  void joint_fault(const std::string& reason) {
    if (!joint_failed_) RCLCPP_ERROR(get_logger(),"J1 latched fault: %s; ID1..4 continue",reason.c_str());
    joint_failed_=true; have_joint_command_=false; ready(false);
  }
  bool competing() {
    return count_publishers("/servo_angle")>1 || count_publishers("/j1_angle")>1
      || count_publishers("/j1_state")>1 || count_publishers("/j1_ready")>1
      || count_publishers("/j1_limits")>1
      || count_publishers("/servo_angle_cmd")>1 || count_publishers("/j1_angle_cmd")>1;
  }
  void check_port_users() {
    struct stat device_stat{};
    if (stat(device_.c_str(),&device_stat)!=0 || !S_ISCHR(device_stat.st_mode))
      throw std::runtime_error("serial device missing/not a character device");
    for (const auto& proc : std::filesystem::directory_iterator("/proc")) {
      const auto name=proc.path().filename().string();
      if (name.empty() || !std::all_of(name.begin(),name.end(),[](char c){return c>='0' && c<='9';})
          || name==std::to_string(getpid())) continue;
      std::error_code ec;
      const auto fd_dir=proc.path()/"fd";
      for (std::filesystem::directory_iterator it(fd_dir,ec),end; !ec && it!=end; it.increment(ec)) {
        struct stat candidate{};
        if (::stat(it->path().c_str(),&candidate)==0 && S_ISCHR(candidate.st_mode)
            && candidate.st_rdev==device_stat.st_rdev)
          throw std::runtime_error("serial port already open by PID "+name);
      }
    }
  }
  void require_tx(int code,const char* operation) {
    if(code!=COMM_SUCCESS) throw std::runtime_error(std::string(operation)+": "+packet_->getTxRxResult(code));
  }
  void write_single_no_reply(int id,uint16_t address,uint16_t length,
                             uint32_t value,const char* operation) {
    if(id<1 || id>252 || (length!=1 && length!=4) || (length==1 && value>255))
      throw std::invalid_argument("invalid single-ID sync write");
    // An ordinary unicast WRITE can produce a Status Packet when Return Level=2.
    // TxOnly does not consume that packet, so a following READ can receive it.
    // Sync Write never produces a Status Packet. Its parameter list contains
    // exactly one motor ID: the broadcast header does NOT command other IDs.
    uint8_t data[5]={static_cast<uint8_t>(id),uint8_t(value),uint8_t(value>>8),
                     uint8_t(value>>16),uint8_t(value>>24)};
    require_tx(packet_->syncWriteTxOnly(port_.get(),address,length,data,length+1),operation);
  }
  uint32_t read_value(int id,uint16_t address,uint16_t length,double timeout) {
    uint8_t bytes[4]{},error=0;
    require_tx(packet_->readTx(port_.get(),id,address,length),"readTx");
    port_->setPacketTimeout(timeout);
    require_tx(packet_->readRx(port_.get(),id,length,bytes,&error),"readRx");
    if(error) throw std::runtime_error(packet_->getRxPacketError(error));
    uint32_t value=0; for(unsigned i=0;i<length;++i) value|=uint32_t(bytes[i])<<(8*i);
    return value;
  }
  void open_bus() {
    check_port_users();
    struct stat st{}; if(stat(device_.c_str(),&st)!=0) throw std::runtime_error("stat failed");
    const std::string lock="/tmp/palletrone_serial_"+std::to_string(st.st_rdev)+".lock";
    lock_fd_=open(lock.c_str(),O_CREAT|O_RDWR,0600);
    if(lock_fd_<0 || flock(lock_fd_,LOCK_EX|LOCK_NB)!=0) throw std::runtime_error("driver lock busy");
    port_.reset(dynamixel::PortHandler::getPortHandler(device_.c_str()));
    packet_=dynamixel::PacketHandler::getPacketHandler(2.0);
    if(!port_->openPort() || !port_->setBaudRate(baud_)) throw std::runtime_error("SDK port open/baud failed");
    exclusive_fd_=open(device_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(exclusive_fd_<0 || ioctl(exclusive_fd_,TIOCEXCL)!=0) throw std::runtime_error("TIOCEXCL failed");
    check_port_users(); // Catch a legacy driver racing startup before any motor writes.
    for(int id=1;id<=4;++id) {
      if(read_value(id,11,1,legacy_timeout_ms_)!=3) throw std::runtime_error("ID1..4 must already use Position mode");
      legacy_goal_[id-1]=read_value(id,132,4,legacy_timeout_ms_);
      if(legacy_goal_[id-1]>4095) throw std::runtime_error("ID1..4 initial position outside single-turn range");
    }
    if(read_value(id_,0,2,joint_timeout_ms_)!=static_cast<uint32_t>(model_)) throw std::runtime_error("J1 model mismatch");
    if(read_value(id_,11,1,joint_timeout_ms_)!=3) throw std::runtime_error("J1 must already use Position mode");
    if(read_value(id_,64,1,joint_timeout_ms_)!=0) throw std::runtime_error("J1 torque must be OFF for safe initialisation");
    const uint32_t current=read_value(id_,132,4,joint_timeout_ms_);
    requested_theta_=commanded_theta_=cal_.angle(static_cast<int32_t>(current));
    // Goal is actual measured position, never an assumed 0 degree/raw tick 0.
    write_single_no_reply(id_,116,4,current,"J1 seed goal");
    write_single_no_reply(id_,64,1,1,"J1 torque enable");
    const uint32_t torque_enabled=read_value(id_,64,1,joint_timeout_ms_);
    if(torque_enabled!=1) throw std::runtime_error("J1 torque enable unconfirmed: read "+std::to_string(torque_enabled));
    for(int id=1;id<=4;++id) {
      write_single_no_reply(id,116,4,legacy_goal_[id-1],"legacy seed goal");
      write_single_no_reply(id,64,1,1,"legacy torque enable");
    }
    read_=std::make_unique<dynamixel::GroupSyncRead>(port_.get(),packet_,132,4);
    write_=std::make_unique<dynamixel::GroupSyncWrite>(port_.get(),packet_,116,4);
    for(int id=1;id<=4;++id) if(!read_->addParam(id)) throw std::runtime_error("sync read registration failed");
    last_joint_=Steady::now(); opened_=true;
    RCLCPP_INFO(get_logger(),"Bus ready; actual J1 angle %.6f rad; allocation unchanged",commanded_theta_);
  }
  void legacy_cycle(Steady::time_point now) {
    if(!legacy_conflict_ && have_legacy_command_ && age(now,legacy_command_at_)<=command_timeout_) {
      write_->clearParam();
      for(int id=1;id<=4;++id) {
        const auto raw=legacy_goal_[id-1];
        uint8_t bytes[4]={uint8_t(raw),uint8_t(raw>>8),uint8_t(raw>>16),uint8_t(raw>>24)};
        if(!write_->addParam(id,bytes)) throw std::runtime_error("sync write registration failed");
      }
      require_tx(write_->txPacket(),"legacy sync write");
    }
    require_tx(read_->txPacket(),"legacy sync read Tx");
    port_->setPacketTimeout(legacy_timeout_ms_);
    require_tx(read_->rxPacket(),"legacy sync read Rx");
    std::array<uint32_t,4> raw{};
    for(int id=1;id<=4;++id) {
      uint8_t error=0;
      read_->getError(id,&error);
      if(!read_->isAvailable(id,132,4) || error)
        throw std::runtime_error("legacy servo read error");
      raw[id-1]=read_->getData(id,132,4);
    }
    std_msgs::msg::Float32MultiArray msg;msg.data=task_flight::legacy_feedback(raw);feedback_->publish(msg);
  }
  void joint_cycle(Steady::time_point now) {
    if(joint_failed_) {ready(false);return;}
    const double dt=std::min(0.1,age(now,last_joint_)); last_joint_=now;
    const auto raw=read_value(id_,132,4,joint_timeout_ms_);
    const double measured=cal_.angle(static_cast<int32_t>(raw));
    if(read_value(id_,64,1,joint_timeout_ms_)!=1) throw std::runtime_error("J1 torque lost");
    if(have_joint_command_ && age(now,joint_command_at_)<=command_timeout_) {
      commanded_theta_+=std::clamp(requested_theta_-commanded_theta_,-rate_*dt,rate_*dt);
      write_single_no_reply(id_,116,4,cal_.raw(commanded_theta_),"J1 goal Tx");
      std_msgs::msg::Float32 goal;goal.data=commanded_theta_;target_->publish(goal);
    }
    std_msgs::msg::Float64MultiArray limits;
    limits.data={cal_.angle_min,cal_.angle_max,rate_}; limits_->publish(limits);
    ready(true);
    std_msgs::msg::Float32 angle;angle.data=measured;angle_->publish(angle);
    sensor_msgs::msg::JointState state;state.header.stamp=get_clock()->now();
    state.name={"j1"};state.position={measured};state_->publish(state);
  }
  void tick() {
    if(dry_ || startup_failed_) return; // No port opens in dry run.
    const auto now=Steady::now();
    if(age(now,born_)<2.0) return;
    if(age(now,last_graph_)>0.5) {
      last_graph_=now;
      if(competing()) {
        if(!opened_) {startup_failed_=true;RCLCPP_ERROR(get_logger(),"Duplicate command/driver endpoint; startup refused");return;}
        joint_fault("duplicate publisher"); legacy_conflict_=true; have_legacy_command_=false;
      }
    }
    if(!opened_) {
      try {open_bus();} catch(const std::exception& e) {startup_failed_=true;ready(false);RCLCPP_ERROR(get_logger(),"Startup refused: %s",e.what());}
      return;
    }
    try {legacy_cycle(now);} catch(const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"ID1..4 I/O: %s",e.what());
    }
    if(age(now,last_joint_)>=joint_period_) {
      try {joint_cycle(now);} catch(const std::exception& e) {joint_fault(e.what());last_joint_=now;}
    }
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Driver>());}
  catch(const std::exception& e) {std::fprintf(stderr,"Driver refused: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
