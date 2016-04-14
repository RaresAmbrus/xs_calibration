#ifndef RBF_KERNEL_3D_H
#define RBF_KERNEL_3D_H

#include <eigen3/Eigen/Dense>
#include <vector>

class rbf_kernel_3d
{
private:
    double sigmaf_sq;
    double l_sq; // length parameter, how far points influence each other
    Eigen::VectorXd p;
public:
    int param_size();
    Eigen::VectorXd& param();
    Eigen::VectorXd param() const;
    void dKdtheta(std::vector<Eigen::ArrayXXd>& Ks, const Eigen::MatrixXd& BV);
    double kernel_function(const Eigen::Vector3d& xi, const Eigen::Vector3d& xj);
    void kernel_dtheta(Eigen::MatrixXd& k_dtheta, const Eigen::Vector3d& x, const Eigen::MatrixXd& BV);
    void kernel_dx(Eigen::MatrixXd& k_dx, const Eigen::VectorXd& x, const Eigen::MatrixXd& BV);
    void kernels_fast(Eigen::ArrayXXd& K_dx, Eigen::ArrayXXd& K_dy, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);
    void construct_covariance_fast(Eigen::MatrixXd& K, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);
    //rbf_kernel(double sigmaf_sq = 1e-0f, double l_sq = 0.05*0.05);
    rbf_kernel_3d(double sigmaf_sq = 10e-0f, double l_sq = 1*1);
};

#endif // RBF_KERNEL_3D_H
