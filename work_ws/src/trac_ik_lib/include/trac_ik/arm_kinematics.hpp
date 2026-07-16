// Copyright (c) 2025, TRACLabs, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef TRAC_IK__ARM_KINEMATICS_HPP_
#define TRAC_IK__ARM_KINEMATICS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "trac_ik/kdl_tl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "trac_ik/visibility_control.hpp"

namespace TRAC_IK
{

/// @brief End-effector pose in Cartesian space.
struct TRAC_IK_PUBLIC Pose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;   // degrees
  double pitch = 0.0;  // degrees
  double yaw = 0.0;    // degrees
};

/// @brief Unified result for kinematics operations.
///
/// On success:         success=true,  approximate=false, joints populated
/// On failure:         success=false, message set
/// On approximate:     success=true,  approximate=true, position_error set
/// On pitch sweep:     success=true,  adjusted_pitch set
struct TRAC_IK_PUBLIC KinResult
{
  bool success = false;
  std::vector<double> joints;       // joint angles in degrees (IK result)
  Pose pose;                        // end-effector pose (FK result)
  std::vector<double> matrix;       // flattened matrix data (Jacobian / pinv result)
  std::string message;
  bool approximate = false;
  double position_error = 0.0;      // meters, meaningful when approximate=true
  double adjusted_pitch = -1.0;     // degrees, set when solution via pitch sweep (<0 means unused)
};

/// @brief Kinematics solver wrapper using TRAC-IK.
///
/// Provides forward kinematics, inverse kinematics (with multi-seed retry,
/// pitch sweep, and approximate fallback), Jacobian computation, damped
/// pseudoinverse, and manipulability measure.
///
/// Designed for 5DOF arms: position (x,y,z) and pitch are tightly controlled;
/// roll and yaw are treated as unconstrained (loose bounds) since a 5DOF arm
/// cannot independently control all 6 task-space dimensions.
class TRAC_IK_PUBLIC ArmKinematics
{
public:
  /// @brief Construct from URDF XML string (provided by caller).
  ///
  /// The caller is responsible for providing a valid URDF XML string
  /// (e.g. read from a .urdf file, or generated from .xacro externally).
  ///
  /// @param urdf_xml     URDF robot description as an XML string.
  /// @param base_link    Name of the base link.
  /// @param tip_link     Name of the end-effector (tip) link.
  /// @param timeout      Default IK timeout in seconds.
  /// @param eps          Default Cartesian error tolerance.
  ArmKinematics(
    const std::string & urdf_xml,
    const std::string & base_link = "base_link",
    const std::string & tip_link = "gripper_tcp",
    double timeout = 0.005,
    double eps = 1e-5);

  ~ArmKinematics() = default;

  /// @brief Number of active joints in the kinematic chain.
  size_t numJoints() const { return num_joints_; }

  /// @brief Joint names in chain order (base → tip).
  const std::vector<std::string> & jointNames() const { return joint_names_; }

  /// @brief Joint limits in degrees, as (lower, upper) pairs.
  const std::vector<std::pair<double, double>> & jointLimits() const { return joint_limits_deg_; }

  // ---- Forward Kinematics ----

  /// @brief Compute forward kinematics.
  /// @param joints_deg  Joint angles in degrees [num_joints].
  /// @return KinResult with pose on success.
  KinResult forwardKinematics(const std::vector<double> & joints_deg) const;

  // ---- Inverse Kinematics ----

  /// @brief Compute inverse kinematics with multi-stage solving.
  ///
  /// Stages:
  ///   1. Exact IK with user seed, then multi-seed retry at original pitch.
  ///   2. Pitch sweep over [pitch - pitch_search, pitch + pitch_search] (1° step).
  ///   3. Approximate fallback using KDL TL solver (if allow_approximate).
  ///
  /// @param x, y, z            Target position (meters).
  /// @param roll, pitch, yaw   Target orientation (degrees). Roll and yaw are
  ///                           treated as soft constraints for 5DOF arms.
  /// @param seed               Initial joint seed (degrees, size num_joints).
  ///                           Empty = use zero seed.
  /// @param n_attempts         Number of random seeds to try (≥1).
  /// @param allow_approximate  If true, return best-effort on total failure.
  /// @param pitch_search       Half-range in degrees for pitch sweep. ≤0 disables.
  /// @return KinResult with joints on success.
  KinResult inverseKinematics(
    double x, double y, double z,
    double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
    const std::vector<double> & seed = {},
    int n_attempts = 10,
    bool allow_approximate = true,
    double pitch_search = 10.0);

  // ---- Jacobian ----

  /// @brief Compute the 6×N geometric Jacobian at the given configuration.
  /// @param joints_deg  Joint angles in degrees.
  /// @return KinResult with a flat vector of 6*N elements (row-major).
  KinResult computeJacobian(const std::vector<double> & joints_deg) const;

  /// @brief Compute damped pseudo-inverse J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹.
  /// @param joints_deg  Joint angles in degrees.
  /// @param damping     Damping factor λ (default 0.01).
  /// @return KinResult with flat vector of N*6 elements (row-major).
  KinResult getJacobianPinv(
    const std::vector<double> & joints_deg, double damping = 0.01) const;

  /// @brief Compute Yoshikawa manipulability μ = √det(JJᵀ).
  /// @param joints_deg  Joint angles in degrees.
  /// @return KinResult float value accessible via .position_error field (temporary).
  ///         >0 = well-conditioned, ≈0 = near singularity.
  double computeManipulability(const std::vector<double> & joints_deg) const;

private:
  // ---- Internal helpers ----

  /// Build KDL chain and extract joint names/limits from URDF XML.
  void buildChain(const std::string & urdf_xml);

  /// Single IK attempt with given seed. Returns true on success.
  bool solveIkOnce(
    const KDL::Frame & target,
    const KDL::JntArray & seed,
    KDL::JntArray & result) const;

  /// Generate diverse seed joint configurations.
  std::vector<KDL::JntArray> generateSeeds(int n) const;

  /// Try approximate IK using KDL TL solver (writes best effort to q_out).
  bool solveIkApproximate(
    const KDL::Frame & target,
    const KDL::JntArray & seed,
    KDL::JntArray & result) const;

  KDL::JntArray toJntArray(const std::vector<double> & joints_deg) const;
  std::vector<double> toDegVector(const KDL::JntArray & q) const;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  // ---- Members ----

  bool initialized_ = false;
  size_t num_joints_ = 0;

  std::string base_link_;
  std::string tip_link_;

  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_limits_deg_;  // degrees

  KDL::Chain chain_;
  KDL::JntArray lb_;   // lower bounds (radians)
  KDL::JntArray ub_;   // upper bounds (radians)

  // TRAC-IK solver for exact IK
  std::unique_ptr<TRAC_IK> ik_solver_;

  // KDL solvers
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> tl_solver_;  // for approximate fallback

  // Default twist bounds for 5DOF arm:
  // Tight: position (xyz) + pitch
  // Loose: roll + yaw (uncontrollable by 5DOF arm)
  KDL::Twist default_bounds_;

  double eps_ = 1e-5;
};

}  // namespace TRAC_IK

#endif  // TRAC_IK__ARM_KINEMATICS_HPP_
