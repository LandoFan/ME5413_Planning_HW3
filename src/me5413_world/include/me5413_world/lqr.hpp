/** lqr.hpp
 *
 * MIT License
 *
 * Implementation of LQR controller
 */

#pragma once

#include <iostream>
#include <Eigen/Dense>

namespace control
{
    class LQR
    {
     public:
        LQR() {};
        LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
        ~LQR() {};

        // Compute control input given state
        Eigen::VectorXd computeControl(const Eigen::VectorXd& state);

     private:
        Eigen::MatrixXd A_;
        Eigen::MatrixXd B_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
    };

    LQR::LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) :
        A_(A),
        B_(B),
        Q_(Q),
        R_(R)
    {};

    Eigen::VectorXd LQR::computeControl(const Eigen::VectorXd& state)
    {
        // Solve Riccati equation
        Eigen::MatrixXd P = Q_;
        Eigen::MatrixXd K;


        for (int i = 0; i < 1000; ++i) // iterate for a maximum of 1000 iterations
        {
            // Compute the control law
            // K = (R_ + B_.transpose() * P * B_).inverse() * (B_.transpose() * P * A_);
            K = (R_ + B_.transpose() * P * B_).inverse() * (B_.transpose() * P * A_);

        // Update the Riccati matrix
            P = Q_ + A_.transpose() * P * (A_ - B_ * K) + K.transpose() * R_ * K;
        }
        std::cout << "K dimensions: " << K.rows() << "x" << K.cols() << std::endl;
        std::cout << "state dimensions: " << state.rows() << "x" << state.cols() << std::endl;
        // Compute control input u = -Kx
        Eigen::VectorXd control_input = -K * state;

        return control_input;
    }
} // namespace control
