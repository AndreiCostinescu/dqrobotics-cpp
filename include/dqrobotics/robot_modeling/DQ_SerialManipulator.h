/**
(C) Copyright 2011-2018 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#ifndef DQ_SerialManipulator_H
#define DQ_SerialManipulator_H

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

class DQ_SerialManipulator: public DQ_Kinematics
{
protected:
    Eigen::MatrixXd    dh_matrix_;
    std::string dh_matrix_convention_;

    Eigen::VectorXd lower_q_limit_;
    Eigen::VectorXd upper_q_limit_;

    DQ curr_effector_;

    virtual DQ _dh2dq(const double& q, const int& ith) const;
    DQ _get_z(const DQ& h) const;

public:
    DQ_SerialManipulator() = delete;
    DQ_SerialManipulator(const Eigen::MatrixXd& dh_matrix, const std::string& convention = "standard");

    Eigen::MatrixXd getDHMatrix();
    Eigen::VectorXd theta() const;
    Eigen::VectorXd d() const;
    Eigen::VectorXd a() const;
    Eigen::VectorXd alpha() const;
    std::string convention() const;

    void set_lower_q_limit(const Eigen::VectorXd& lower_q_limit);
    Eigen::VectorXd lower_q_limit() const;

    void set_upper_q_limit(const Eigen::VectorXd& upper_q_limit);
    Eigen::VectorXd upper_q_limit() const;

    DQ effector() const;
    DQ set_effector(const DQ& new_effector);

    DQ raw_fkm(const Eigen::VectorXd& q_vec) const;
    virtual DQ raw_fkm(const Eigen::VectorXd& q_vec, const int& to_ith_link) const;

    DQ fkm(const Eigen::VectorXd& q_vec) const override; //Override from DQ_Kinematics
    DQ fkm(const Eigen::VectorXd& q_vec, const int& to_ith_link) const override;

    Eigen::MatrixXd raw_pose_jacobian(const Eigen::VectorXd& q_vec) const;
    virtual Eigen::MatrixXd raw_pose_jacobian(const Eigen::VectorXd& q_vec, const int& to_ith_link) const;

    Eigen::MatrixXd pose_jacobian_derivative(const Eigen::VectorXd& q_vec, const Eigen::VectorXd& q_vec_dot) const;
    Eigen::MatrixXd pose_jacobian_derivative(const Eigen::VectorXd& q_vec, const Eigen::VectorXd& q_vec_dot, const int& to_ith_link) const;

    int get_dim_configuration_space() const override; //Override from DQ_Kinematics

    Eigen::MatrixXd pose_jacobian(const Eigen::VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics
    Eigen::MatrixXd pose_jacobian(const Eigen::VectorXd& q_vec) const override; //Override from DQ_Kinematics

};

}//Namespace DQRobotics

#endif // DQ_SerialManipulator_H
