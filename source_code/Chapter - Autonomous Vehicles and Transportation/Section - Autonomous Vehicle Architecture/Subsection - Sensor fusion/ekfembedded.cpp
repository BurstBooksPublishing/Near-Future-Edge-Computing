#include 
#include 

using namespace Eigen;
using steady_clock = std::chrono::steady_clock;

// State: [x,y,z,vx,vy,vz] (6D)
struct EKF6 {
  // fixed-size matrices for deterministic stack allocation
  Matrix x;               // state
  Matrix P;               // covariance
  Matrix Q;               // process noise
  EKF6() { P.setIdentity(); Q.setIdentity()*1e-3; x.setZero(); }

  // predict with constant velocity model dt seconds
  void predict(double dt) {
    Matrix F = Matrix::Identity();
    F(0,3)=dt; F(1,4)=dt; F(2,5)=dt;
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  // measurement model: position observation z = H x + noise
  void update_position(const Matrix& z,
                       const Matrix& R) {
    Matrix H = Matrix::Zero();
    H(0,0)=1; H(1,1)=1; H(2,2)=1;
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse(); // Kalman gain
    Matrix y = z - H * x;
    x += K * y;
    P = (Matrix::Identity() - K * H) * P;
  }
};