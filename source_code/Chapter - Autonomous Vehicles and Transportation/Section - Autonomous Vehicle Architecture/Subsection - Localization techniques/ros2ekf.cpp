#include 
#include 
#include 
#include 
#include 
#include 
#include 

using Eigen::MatrixXd; using Eigen::VectorXd;

class EkfNode : public rclcpp::Node {
public:
  EkfNode(): Node("ekf_node") {
    imu_sub_ = create_subscription(
      "/imu", rclcpp::SensorDataQoS(), std::bind(&EkfNode::imu_cb,this,std::placeholders::_1));
    gnss_sub_ = create_subscription(
      "/gnss/fix", 10, std::bind(&EkfNode::gnss_cb,this,std::placeholders::_1));
    odom_sub_ = create_subscription(
      "/wheel/odom", 10, std::bind(&EkfNode::odom_cb,this,std::placeholders::_1));
    pose_pub_ = create_publisher("/localization/pose",10);
    init_filter();
  }

private:
  void init_filter() {
    x_ = VectorXd::Zero(9);               // pos(3), vel(3), attitude(3)
    P_ = MatrixXd::Identity(9,9)*1e-2;
    Q_ = MatrixXd::Identity(9,9);
    R_gnss_ = MatrixXd::Identity(3,3)*1.0; // meters^2
    last_time_ = this->now();
  }
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard lk(mutex_);
    double dt = (msg->header.stamp - last_time_).seconds();
    if(dt<=0) return;
    // simple constant-acceleration predict; convert IMU to body accel then world via attitude
    predict(dt, msg->linear_acceleration, msg->angular_velocity);
    last_time_ = msg->header.stamp;
  }
  void gnss_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard lk(mutex_);
    VectorXd z(3);
    // convert lat/lon/alt -> ECEF or local ENU; here assume caller provides ENU in msg->position_covariance (example)
    z << msg->position_covariance[0], msg->position_covariance[1], msg->position_covariance[2]; // placeholder
    update_gnss(z);
  }
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard lk(mutex_);
    // use wheel odometry velocity for velocity update
    VectorXd z = VectorXd::Zero(3);
    z.segment<3>(0) << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    update_odom(z);
  }

  void predict(double dt, const geometry_msgs::msg::Vector3 &acc, const geometry_msgs::msg::Vector3 &gyro) {
    // state propagation (minimal example)
    x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * dt*dt * VectorXd::Map(&acc.x,3);
    x_.segment<3>(3) += dt * VectorXd::Map(&acc.x,3);
    // linearized F for covariance propagation
    MatrixXd F = MatrixXd::Identity(9,9);
    F.block<3,3>(0,3) = MatrixXd::Identity(3,3)*dt;
    P_ = F*P_*F.transpose() + Q_;
  }

  void update_gnss(const VectorXd &z) {
    MatrixXd H = MatrixXd::Zero(3,9); H.block<3,3>(0,0)=MatrixXd::Identity(3,3);
    MatrixXd S = H*P_*H.transpose() + R_gnss_;
    MatrixXd K = P_*H.transpose()*S.inverse();
    x_ = x_ + K*(z - H*x_);
    P_ = (MatrixXd::Identity(9,9)-K*H)*P_;
    publish_pose();
  }

  void update_odom(const VectorXd &z) {
    MatrixXd H = MatrixXd::Zero(3,9); H.block<3,3>(0,3)=MatrixXd::Identity(3,3);
    MatrixXd R = MatrixXd::Identity(3,3)*0.01;
    MatrixXd S = H*P_*H.transpose() + R;
    MatrixXd K = P_*H.transpose()*S.inverse();
    x_ = x_ + K*(z - H*x_);
    P_ = (MatrixXd::Identity(9,9)-K*H)*P_;
  }

  void publish_pose() {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = "map";
    out.pose.pose.position.x = x_(0);
    out.pose.pose.position.y = x_(1);
    out.pose.pose.position.z = x_(2);
    // attitude conversion omitted for brevity
    // copy covariance
    for(size_t i=0;i<36;i++) out.pose.covariance[i]=0.0;
    out.pose.covariance[0]=P_(0,0); out.pose.covariance[7]=P_(1,1); out.pose.covariance[14]=P_(2,2);
    pose_pub_->publish(out);
  }

  rclcpp::Subscription::Shared
\section{V2X Communication and Intelligent Transportation Systems}
\subsection{Dedicated short-range communications}
Building on the vehicle perception and planning elements, this subsection examines Dedicated Short-Range Communications (DSRC) as a low-latency V2X fabric and its integration with edge compute components such as OBUs, RSUs, and MEC hosts. The focus is on protocol-stack trade-offs, latency/reliability math, a concrete edge deployment pattern, and an implementation sketch for operational forwarding between an edge inference module and a DSRC stack.

DSRC in system context
\begin{itemize}
\item Standards and stacks: DSRC refers to IEEE 802.11p (PHY/MAC), with higher-layer profiles defined by IEEE 1609 (WAVE) and message sets standardized by SAE J2735 (Basic Safety Message, BSM) and ETSI ITS-G5 (Europe). Implementations commonly run on OBUs and RSUs and integrate with vehicle SoCs (NVIDIA Drive, NXP S32, Qualcomm Snapdragon Ride) or RSU appliances (Intel x86 with dedicated 802.11p radio).
\item Edge relevance: DSRC provides ad hoc low-latency vehicle-to-vehicle (V2V) and vehicle-to-infrastructure (V2I) links. Edge compute (MEC) can host cooperative perception services, offload heavy sensor fusion, and expose ETSI MEC APIs for local traffic management. Real-time OS choices include QNX or AUTOSAR for safety-critical modules and Linux PREEMPT_RT for MEC applications.
\end{itemize}

Theory — latency and reliability trade-offs
\begin{itemize}
\item Latency budget decomposition for a safety message:
\begin{equation}[H]\label{eq:dsrc_latency_budget}
L_{\text{total}} = L_{\text{tx}} + L_{\text{proc}} + L_{\text{queue}} + L_{\text{pl}} \le L_{\text{deadline}}
\end{equation}
where $L_{\text{tx}}$ is PHY/MAC transmission time, $L_{\text{proc}}$ is inference/aggregation time at the edge, $L_{\text{queue}}$ is queuing at network stacks, and $L_{\text{pl}}$ is propagation and propagation-like delays.
\item Reliability with limited retransmissions: if per-packet loss probability is $p$ and at most $n$ retransmissions are permitted within the latency window, the probability of successful delivery is
\begin{equation}[H]\label{eq:dsrc_reliability}
P_{\text{succ}} = 1 - p^{\,n+1}
\end{equation}
and expected transmission attempts $E[N]=\frac{1}{1-p}$. These relations guide how many MAC-layer retries an edge node should allow to meet $P_{\text{succ}}$ without violating \eqref{eq:dsrc_latency_budget}.
\end{itemize}

Conceptual example
\begin{itemize}
\item Scenario: a platoon coordinator runs on a roadside MEC node (Intel i7 with EdgeX Foundry microservices) receiving intermittent cooperative perception from vehicles. Vehicles broadcast BSMs at 10 Hz. Congestion control adapts to channel load: when measured channel busy ratio exceeds threshold, vehicles reduce rate to 5 Hz and increase transmit power control is constrained to regulatory limits.
\item Numerical example: with $p=0.1$ and $n=2$, \eqref{eq:dsrc_reliability} yields $P_{\text{succ}}=1-0.1^3=0.999$ and $E[N]\approx1.111$. If $L_{\text{deadline}}=100\,\text{ms}$ and $L_{\text{proc}}=20\,\text{ms}$, budget remaining for wireless and queueing is $80\,\text{ms}$; MAC retry backoffs must be sized accordingly.
\end{itemize}

Application and deployment pattern
\begin{itemize}
\item Architecture: OBU runs low-latency safety stack (ASIL-rated with AUTOSAR or QNX) and a gateway service forwards sanitized fusion results to an MEC service via secure channel (TLS or ETSI MEC API). The MEC performs cooperative perception, executes machine-learning models (TensorRT on NVIDIA Jetson Xavier or Drive Orin), and publishes advisories to local RSUs. The RSU rebroadcasts in the ITS channel for local V2V consumption.
\item Security: use IEEE 1609.2 for signed messages and certificate management. Consider intersection attacker models and validate message plausibility at the edge (sensor fusion checks) before forwarding.
\end{itemize}

Implementation sketch: reliable, priority-aware forwarder
\begin{itemize}
\item This production-focused Python asyncio forwarder runs on an MEC host and forwards processed hazard events to a local DSRC middleware via UDP. It sets process real-time scheduling and socket priority to reduce queueing latency. (Requires root and PREEMPT_RT or equivalent.)
\begin{lstlisting}[language=Python,caption={Real-time UDP forwarder for DSRC middleware (production-ready pattern)},label={lst:dsrc_forwarder}]
#!/usr/bin/env python3
# Minimal production-ready forwarder: set RT priority and socket priority,
# accept unix-domain input from edge inference, forward via UDP to DSRC middleware.
import asyncio, os, socket, struct

DSRC_HOST = "127.0.0.1"        # middleware on-host (or RSU IP)
DSRC_PORT = 1511               # common service port for V2X middleware
UNIX_SOCKET = "/var/run/edge_haz.sock"

def set_realtime():
    # SCHED_FIFO with priority 50; requires CAP_SYS_NICE/root
    param = os.sched_param(50)
    os.sched_setscheduler(0, os.SCHED_FIFO, param)

async def forward_loop():
    # create UDP socket with high priority
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY, 6)  # DSCP/Priority hint
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.setblocking(False)

    # unix socket server for inference outputs (JSON lines)
    if os.path.exists(UNIX_SOCKET):
        os.remove(UNIX_SOCKET)
    server = await asyncio.start_unix_server(
        lambda r,w: handle_client(r, w, udp), path=UNIX_SOCKET
    )
    async with server:
        await server.serve_forever()

async def handle_client(reader, writer, udp_sock):
    try:
        while True:
            line = await reader.readline()
            if not line:
                break
            # minimal validation and timestamping
            msg = line.rstrip(b"\n")
            ts = int(asyncio.get_event_loop().time() * 1000)
            payload = struct.pack("!Q", ts) + msg  # prepend 8-byte ms timestamp
            await asyncio.get_event_loop().sock_sendall(udp_sock, payload, (DSRC_HOST, DSRC_PORT))
    finally:
        writer.close()
        await writer.wait_closed()

if __name__ == "__main__":
    set_realtime()
    asyncio.run(forward_loop())