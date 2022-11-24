/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedMotionMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:
    double mass;
    Eigen::Matrix3d IPntFc_F;                           //!< [kg-m^2] Inertia of prescribed body about point Fc in F frame components
    Eigen::Vector3d r_MB_B;                             //!< [m] position vector of point M relative to point B in B frame components
    Eigen::Vector3d r_FcF_F;                            //!< [m] position vector of point Fc relative to point F in F frame components
    Eigen::Vector3d omega_MB_B;
    Eigen::Vector3d omegaPrime_MB_B;
    Eigen::MRPd sigma_MB;                               //!< MRP attitude of frame M relative to the B frame

    // Prescribed parameters
    Eigen::Vector3d r_FM_M;                             //!< [m] position vector of point F relative to point M in M frame components
    Eigen::Vector3d rPrime_FM_M;                        //!< [m/s] B frame time derivative of position vector r_FM_M in M frame components
    Eigen::Vector3d rPrimePrime_FM_M;                   //!< [m/s^2] B frame time derivative of vector rPrime_FM_M in M frame components
    Eigen::Vector3d omega_FM_F;                         //!< [rad/s] angular velocity vector of F frame relative to the B frame in F frame components
    Eigen::Vector3d omegaPrime_FM_F;                    //!< [rad/s^2] B frame time derivative of omega_FB_F in F frame components
    Eigen::MRPd sigma_FM;                           //!< Initial MRP attitude of frame F relative to the M frame

    ReadFunctor<PrescribedMotionMsgPayload> prescribedMotionInMsg; //!< -- (optional) motor torque input message name
    Message<PrescribedMotionMsgPayload> prescribedMotionOutMsg;     //!< state output message
    Message<SCStatesMsgPayload> prescribedMotionConfigLogOutMsg;    //!< state config log message
    BSKLogger bskLogger;                                            //!< -- BSK Logging

private:
    static uint64_t effectorID;                                     //!< [] ID number of this panel

    // Given quantities from user in python
    Eigen::Matrix3d IPntFc_B;                           //!< [kg-m^2] Inertia of prescribed body about point Fc in B frame components
    Eigen::Vector3d r_FB_B;                             //!< [m] position vector of point F relative to point B in B frame components
    Eigen::Vector3d r_FcF_B;                            //!< [m] position vector of point Fc relative to point F in B frame components

    // Prescribed parameters in body frame components
    Eigen::Vector3d r_FM_B;                             //!< [m] position vector of point F relative to point M in B frame components
    Eigen::Vector3d rPrime_FM_B;                        //!< [m/s] B frame time derivative of position vector r_FM_B
    Eigen::Vector3d rPrimePrime_FM_B;                   //!< [m/s^2] B frame time derivative of rPrime_FM_B
    Eigen::Vector3d omega_FM_B;                         //!< [rad/s] angular velocity vector of F frame relative to the B frame in B frame components
    Eigen::Vector3d omegaPrime_FM_B;                    //!< [rad/s^2] B frame time derivative of omega_FB_B in B frame components

    Eigen::Vector3d omega_FB_B;                         //!< [rad/s] angular velocity vector of F frame relative to the B frame in B frame components
    Eigen::Vector3d omegaPrime_FB_B;                    //!< [rad/s^2] B frame time derivative of omega_FB_B in B frame components

    // Other vector quantities
    Eigen::Vector3d r_FcM_B;                            //!< [m] position vector of F frame center of mass relative to point M in B frame components
    Eigen::Vector3d r_FcB_B;                            //!< [m] position vector of F frame center of mass relative to point B in B frame components
    Eigen::Vector3d rPrime_FcM_B;                       //!< [m/s] B frame time derivative of position vector r_FcM_B in B frame components
    Eigen::Vector3d rPrime_FcB_B;                       //!< [m/s] B frame time derivative of position vector r_FcM_B in B frame components
    Eigen::Vector3d rPrimePrime_FcB_B;                  //!< [m/s^2] B frame time derivative of rPrime_FcB_B in B frame components
    Eigen::Vector3d omega_BN_B;                         //!< [rad/s] angular velocity vector of B frame relative to the inertial frame in B frame components
    Eigen::Vector3d omega_FN_B;                         //!< [rad/s] angular velocity vector of F frame relative to the inertial frame in B frame components
    Eigen::Vector3d rDot_FcB_B;                         //!< [m/s] inertial time derivative of position vector r_FcB_B in B frame components
    Eigen::MRPd sigma_BN;                               //!< MRP attitude of B frame relative to the inertial frame

    // DCMs
    Eigen::Matrix3d dcm_BN;                             //!< DCM from inertial frame to B frame
    Eigen::Matrix3d dcm_BM;                             //!< DCM from M frame to B frame
    Eigen::Matrix3d dcm_FM;                             //!< DCM from M frame to F frame
    Eigen::Matrix3d dcm_BF;                             //!< DCM from F frame to B frame

    // Other matrix quantities
    Eigen::Matrix3d rTilde_FcB_B;                       //!< [m] tilde matrix of r_FcB_B in B frame components
    Eigen::Matrix3d omegaTilde_BN_B;                    //!< [rad/s] tilde matrix of omega_BN_B in B frame components
    Eigen::Matrix3d omegaTilde_FB_B;                    //!< [rad/s] tilde matrix of omega_FB_B in B frame components

    // Effector body properties
    Eigen::Vector3d r_FcN_N;                            //!< [m] position vector of F frame center of mass relative to the inertial frame origin in inertial frame components
    Eigen::Vector3d v_FcN_N;                            //!< [m/s] inertial velocity vector of Fc relative to inertial frame in inertial frame components
    Eigen::Vector3d sigma_FN;                           //!< MRP attitude of frame F relative to the inertial frame
    Eigen::Vector3d omega_FN_F;                         //!< [rad/s] angular velocity vector of frame F relative to the inertial frame in F frame components

    // States
    StateData *hubSigma;                                //!< hub/inertial attitude represented by MRP
    StateData *hubOmega;                                //!< hub/inertial angular velocity vector in B frame components
    StateData *hubPosition;                             //!< hub/inertial position vector in inertial frame components
    StateData *hubVelocity;                             //!< hub/inertial velocity vector in inertial frame components
    Eigen::MatrixXd *c_B;                               //!< [m] vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;                          //!< [m/s] body time derivative of vector c_B in B frame components

public:
    PrescribedMotionStateEffector();                        //!< -- Constructor
    ~PrescribedMotionStateEffector();                       //!< -- Destructor
    void Reset(uint64_t CurrentClock);                      //!< -- Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock);   //!< -- Method for writing the output messages
	void UpdateState(uint64_t CurrentSimNanos);             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn);         //!< -- Method for registering the effector's states
    void linkInStates(DynParamManager& states);             //!< -- Method for getting access to other states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);                         //!< -- Method for effector to compute its derivatives
    void updateEffectorMassProps(double integTime);         //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B);       //!< -- Method for computing energy and momentum for effectors
    void prependSpacecraftNameToStates();                   //!< Method used for multiple spacecraft
    void computePrescribedMotionInertialStates();           //!< Method for computing the effector's states
};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
