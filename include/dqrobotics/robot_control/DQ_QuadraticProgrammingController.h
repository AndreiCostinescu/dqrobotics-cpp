/**
(C) Copyright 2019 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#ifndef DQ_ROBOT_CONTROL_DQ_TASKSPACEQUADRATICPROGRAMMINGCONTROLLER_H
#define DQ_ROBOT_CONTROL_DQ_TASKSPACEQUADRATICPROGRAMMINGCONTROLLER_H

#include<dqrobotics/robot_control/DQ_KinematicConstrainedController.h>
#include<dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

namespace DQ_robotics
{
class DQ_QuadraticProgrammingController:public DQ_KinematicConstrainedController
{
protected:
    //Only observer no shared ownership
    DQ_QuadraticProgrammingSolver* qp_solver_;

    DQ_QuadraticProgrammingController(DQ_Kinematics *robot, DQ_QuadraticProgrammingSolver *solver);
public:
    //Remove default constructor
    DQ_QuadraticProgrammingController()=delete;

    virtual Eigen::MatrixXd compute_objective_function_symmetric_matrix(const Eigen::MatrixXd& J, const Eigen::VectorXd& task_error)=0;
    virtual Eigen::VectorXd compute_objective_function_linear_component(const Eigen::MatrixXd& J, const Eigen::VectorXd& task_error)=0;

    virtual Eigen::VectorXd compute_setpoint_control_signal(const Eigen::VectorXd&q, const Eigen::VectorXd& task_reference) override;
    virtual Eigen::VectorXd compute_tracking_control_signal(const Eigen::VectorXd&q, const Eigen::VectorXd& task_reference, const Eigen::VectorXd& feed_forward) override;

};
}

#endif
