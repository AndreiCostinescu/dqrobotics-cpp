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

#ifndef DQ_KINEMATICS_H
#define DQ_KINEMATICS_H

#include <dqrobotics/DQ.h>

#include <math.h>       //library for math functions
#include <stdexcept>    //For range_error
#include <eigen3/Eigen/Dense>  //Library for matrix usage
#include <limits>       //Used in pseudoinverse()
#include <string>

namespace DQ_robotics
{



class DQ_kinematics{

    // private attributtes
private:

    //Para uso nas funções Jacobian...
    Eigen::MatrixXd    dh_matrix_;
    std::string dh_matrix_convention_;

    DQ curr_base_;
    DQ curr_effector_;

    // public methods
public:
    // Class constructors: Creates a Dual Quaternion as a DQ object.

    DQ_kinematics(const Eigen::MatrixXd& dh_matrix, const std::string& convention = "standard" );

    DQ_kinematics(){};

    ~DQ_kinematics();


    /*
        * Public constant methods: Can be called by DQ_kinematics objects.
        * To use these methods, type: 'dq_kinematics_object.method_name();' where 'method_name' is the name of one of the methods below.
        * Or in another way type: 'DQ_kinematics::method_name(dq_kinematics_object);' that works well too.
        * For displaying the results of methods, the DISPLAY and MATRIX functions of DQ class can be used
        */

    Eigen::MatrixXd getDHMatrix();

    int n_links() const;

    Eigen::VectorXd theta() const;

    Eigen::VectorXd d() const;

    Eigen::VectorXd a() const;

    Eigen::VectorXd alpha() const;

    Eigen::VectorXd dummy() const;
    void set_dummy( const Eigen::VectorXd& dummy_vector);

    int n_dummy() const;

    std::string convention() const;

    DQ base() const;
    DQ set_base( const DQ& new_base);

    DQ effector() const;
    DQ set_effector( const DQ& new_effector);

    DQ raw_fkm( const Eigen::VectorXd& theta_vec) const;
    DQ raw_fkm( const Eigen::VectorXd& theta_vec, const int& ith) const;

    DQ fkm( const Eigen::VectorXd& theta_vec) const;
    DQ fkm( const Eigen::VectorXd& theta_vec, const int& ith) const;

    DQ dh2dq( const double& theta_ang, const int& link_i) const;

    DQ get_z( const Eigen::VectorXd& q) const;

    Eigen::MatrixXd pose_jacobian           ( const Eigen::VectorXd& theta_vec, const int& to_link) const;
    Eigen::MatrixXd pose_jacobian           ( const Eigen::VectorXd& theta_vec) const;
    Eigen::MatrixXd raw_pose_jacobian       ( const Eigen::VectorXd& theta_vec, const int& to_link) const;
    Eigen::MatrixXd pose_jacobian_derivative( const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& theta_vec_dot, const int& to_link) const;

    ///DEPRECATED SIGNATURES
    DEPRECATED Eigen::MatrixXd analyticalJacobian( const Eigen::VectorXd& theta_vec) const;
    DEPRECATED Eigen::MatrixXd jacobian(           const Eigen::VectorXd& theta_vec, const int& to_link) const;
    DEPRECATED Eigen::MatrixXd jacobian(           const Eigen::VectorXd& theta_vec) const;
    DEPRECATED Eigen::MatrixXd raw_jacobian(       const Eigen::VectorXd& theta_vec, const int& to_link) const;
    DEPRECATED Eigen::MatrixXd jacobianDerivative( const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& theta_vec_dot, const int& to_link) const;
    DEPRECATED int links() const;
};


int n_links( const DQ_kinematics& dq_kin);

Eigen::VectorXd theta( const DQ_kinematics& dq_kin);

Eigen::VectorXd d( const DQ_kinematics& dq_kin);

Eigen::VectorXd a( const DQ_kinematics& dq_kin);

Eigen::VectorXd alpha( const DQ_kinematics& dq_kin);

Eigen::VectorXd dummy( const DQ_kinematics& dq_kin);

int n_dummy( const DQ_kinematics& dq_kin);

std::string convention( const DQ_kinematics& dq_kin);

DQ base( const DQ_kinematics& dq_kin);

DQ effector( const DQ_kinematics& dq_kin);

DQ set_base( DQ_kinematics& dq_kin, const DQ& new_base);

DQ set_effector( DQ_kinematics& dq_kin, const DQ& new_effector);

DQ raw_fkm( const DQ_kinematics& dq_kin, const Eigen::VectorXd& theta_vec);
DQ raw_fkm( const DQ_kinematics& dq_kin, const Eigen::VectorXd& theta_vec, const int& ith);

DQ dh2dq( const DQ_kinematics& dq_kin, const double& theta_ang, const int& link_i);

DQ get_z( const DQ_kinematics& dq_kin, const Eigen::VectorXd& q);

Eigen::MatrixXd pose_jacobian(            const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec );
Eigen::MatrixXd pose_jacobian(            const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec, const int &to_link);
Eigen::MatrixXd raw_pose_jacobian(        const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec, const int& to_link);
Eigen::MatrixXd pose_jacobian_derivative( const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& theta_vec_dot, const int& to_link);

Eigen::MatrixXd rotation_jacobian   ( const Eigen::MatrixXd& pose_jacobian);

Eigen::MatrixXd translation_jacobian( const Eigen::MatrixXd& pose_jacobian, const DQ& x);

Eigen::MatrixXd line_jacobian       ( const Eigen::MatrixXd& pose_jacobian, const DQ& pose, const DQ& line_direction);

Eigen::MatrixXd plane_jacobian      ( const Eigen::MatrixXd& pose_jacobian, const DQ& pose, const DQ& plane_normal);

Eigen::MatrixXd point_to_point_distance_jacobian(const Eigen::MatrixXd& translation_jacobian, const DQ& robot_point_translation, const DQ& workspace_point_translation);
double   point_to_point_residual         (const DQ& robot_point_translation, const DQ& workspace_point_translation, const DQ& workspace_point_translation_derivative);

Eigen::MatrixXd point_to_line_distance_jacobian (const Eigen::MatrixXd& translation_jacobian, const DQ& robot_point_translation, const DQ& workspace_line);
double   point_to_line_residual          (const DQ& robot_point_translation, const DQ& workspace_line, const DQ& workspace_line_derivative);

Eigen::MatrixXd point_to_plane_distance_jacobian(const Eigen::MatrixXd& translation_jacobian, const DQ& robot_point_translation, const DQ& workspace_plane);
double   point_to_plane_residual         (const DQ& translation, const DQ& plane_derivative);

Eigen::MatrixXd line_to_point_distance_jacobian (const Eigen::MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_point_translation);
double   line_to_point_residual          (const DQ& robot_line, const DQ& workspace_point_translation, const DQ& workspace_point_translation_derivative);

Eigen::MatrixXd line_to_line_distance_jacobian  (const Eigen::MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_line);
double   line_to_point_residual          (const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative);

Eigen::MatrixXd plane_to_point_distance_jacobian(const Eigen::MatrixXd& plane_jacobian, const DQ& robot_plane, const DQ& workspace_point);
double   plane_to_point_residual         (const DQ& robot_plane, const DQ& workspace_point_derivative);

//Eigen::MatrixXd distance_jacobian( const DQ_kinematics& dq_kin, const Eigen::MatrixXd& param_jacobian, const DQ& x);

Eigen::MatrixXd pseudo_inverse( const Eigen::MatrixXd& matrix);

///DEPRECATED SIGNATURES
DEPRECATED Eigen::MatrixXd analyticalJacobian( const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec);
DEPRECATED Eigen::MatrixXd jacobian(           const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec,  const int &to_link);
DEPRECATED Eigen::MatrixXd jacobian(           const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec); //The MATLAB syntax, kept for legacy reasons.
DEPRECATED Eigen::MatrixXd jacobianDerivative( const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& theta_vec_dot, const int& to_link);
DEPRECATED Eigen::MatrixXd raw_jacobian(       const DQ_kinematics& dq_kin,   const Eigen::VectorXd& theta_vec, const int& to_link);
DEPRECATED Eigen::MatrixXd rotationJacobian(   const Eigen::MatrixXd& pose_jacobian);
DEPRECATED Eigen::MatrixXd translationJacobian(const Eigen::MatrixXd& pose_jacobian, const DQ& x);
DEPRECATED Eigen::MatrixXd jacobp(             const Eigen::MatrixXd& pose_jacobian, const DQ& x); //The MATLAB syntax, kept for legacy reasons.
//DEPRECATED Eigen::MatrixXd distanceJacobian(   const Eigen::MatrixXd& param_jacobian, const DQ& x);
//DEPRECATED Eigen::MatrixXd jacobd(             const Eigen::MatrixXd& param_jacobian, const DQ& x);
DEPRECATED Eigen::MatrixXd pseudoInverse(      const Eigen::MatrixXd& matrix);
DEPRECATED int      links(              const DQ_kinematics& dq_kin);

}//Namespace DQRobotics

#endif // DQ_KINEMATICS_H_INCLUDED
