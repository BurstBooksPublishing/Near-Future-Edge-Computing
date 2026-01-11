#include 
#include 
#include 
#include 
#include 
#include 
#include 
#include 

// Real-time scheduling helper
void set_realtime_priority(int prio) {
  struct sched_param p; p.sched_priority = prio;
  if (sched_setscheduler(0, SCHED_FIFO, &p) != 0) { perror("sched_setscheduler"); }
}

class LQRController : public rclcpp::Node {
public:
  LQRController()
  : Node("lqr_controller") {
    // Precomputed gain and estimator matrices (loaded from secure storage)
    K_.resize(2,4); // example: 2 inputs, 4 states
    // ... fill K_ with offline values ...
    // Open SocketCAN interface for actuators
    ifreq ifr; sockaddr_can addr{};
    sock_ = socket(PF_CAN, SOCK_DGRAM, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(sock_, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock_, (struct sockaddr*)&addr, sizeof(addr));
  }

  void run_loop() {
    set_realtime_priority(80); // request high priority
    rclcpp::Rate rate(100.0); // 100 Hz loop
    while (rclcpp::ok()) {
      // Acquire latest state estimate from fusion module (non-blocking)
      Eigen::Vector4d x = read_state(); // placeholder for sensor fusion API
      Eigen::Vector2d u = -K_ * x; // discrete LQR law
      // Enforce actuator limits safely
      saturate(u);
      send_actuator_can(u); // pack into CAN frames
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
  }

private:
  int sock_;
  Eigen::MatrixXd K_;
  Eigen::Vector4d read_state() {
    Eigen::Vector4d x; x.setZero();
    // Replace with fast IPC (shared memory) from perception/fusion
    return x;
  }
  void saturate(Eigen::Vector2d &u) {
    // Simple clamp to actuator physical limits
    for (int i=0;i(u[0]*10000);
    int16_t v1 = static_cast(u[1]*10000);
    std::memcpy(frame.data, &v0, 2); std::memcpy(frame.data+2,&v1,2);
    send(sock_, &frame, sizeof(frame), 0);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared();
  node->run_loop();
  rclcpp::shutdown();
  return 0;
}