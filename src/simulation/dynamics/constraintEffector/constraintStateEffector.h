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


#ifndef CONSTRAINT_STATE_EFFECTOR_H
#define CONSTRAINT_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>



/*! @brief thruster dynamic effector class */
class ConstraintStateEffector: public StateEffector, public SysModel {
public:
    ConstraintStateEffector();
    ~ConstraintStateEffector();
    void Reset(uint64_t CurrentSimNanos);
    bool ReadInputs();
    void writeOutputStateMessages(uint64_t CurrentClock);
    void registerStates(DynParamManager& states);  //!< -- Method for the effector to register its states
    void linkInStates(DynParamManager& states);  //!< -- Method for the effector to get access of other states
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);
    void updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< Method to pass the forces and torques onto the hub
    void updateEffectorMassProps(double integTime);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    // Gains for PD controller
    double alpha;
    double beta;
    Eigen::Matrix3d K;
    Eigen::Matrix3d P;

    // State information
    std::vector<double> kInit;                //!< [] Vector of initial thruster states
    std::string nameOfKState;    //!< -- Identifier for the kappa state data container

    // Forces and torques
    Eigen::Vector3d forceOnBody1_B1;
    Eigen::Vector3d forceOnBody2_B2;
    Eigen::Vector3d torqueOnBody1PntB1_B1;
    Eigen::Vector3d torqueOnBody2PntB2_B2;

private:
    static uint64_t effectorID;    //!< [] ID number of this panel

    double c;
    double p;

    // State structures
    StateData* hubSigma;        //!< class variable
    StateData* hubOmega;        //!< class varaible
    StateData* kState;      //!< -- state manager of theta for hinged rigid body
    BSKLogger bskLogger;        //!< -- BSK Logging
};


#endif /* THRUSTER_STATE_EFFECTOR_H */
