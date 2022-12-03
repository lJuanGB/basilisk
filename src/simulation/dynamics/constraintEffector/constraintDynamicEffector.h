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


#ifndef CONSTRAINT_DYNAMIC_EFFECTOR_H
#define CONSTRAINT_DYNAMIC_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>

/*! @brief thruster dynamic effector class */
class ConstraintDynamicEffector: public SysModel, public DynamicEffector {
public:
    ConstraintDynamicEffector();
    ~ConstraintDynamicEffector();
    void linkInStates(DynParamManager& states, uint64_t spacecraftID);
    void computeForceTorque(double integTime, double timeStep, uint64_t spacecraftID);
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    

public:   
    // Gains for PD controller
    double alpha;
    double beta;
    double kI;
    double kI_att;
    double K;
    double P;

    // Constraint length and direction
    Eigen::Vector3d r_P1B1_B1; // position vector from spacecraft 1 hub to its connection point P1
    Eigen::Vector3d r_P2B2_B2; // position vector from spacecraft 2 hub to its connection point P2
    Eigen::Vector3d r_P2P1_B1Init; // precribed position vector from spacecraft 1 connection point to spacecraft 2 connection point

    Eigen::Vector3d psi_N;
    Eigen::Vector3d psi_B1;
    Eigen::Vector3d psiPrime_N;
    Eigen::Vector3d psiPrime_B1;
    Eigen::Vector3d sigma_B2B1;
    Eigen::Vector3d omega_B2B1_B2;

	std::vector<StateData*> hubSigma;                           // class variable
    std::vector<StateData*> hubOmega;                           // class varaible
    std::vector<StateData*> hubPosition;                        // class variable
    std::vector<StateData*> hubVelocity;                        // class varaible
    BSKLogger bskLogger;                      //BSK Logging

private:
    // Spacecraft order flag
    int scCounterFlag;
    // Spacecraft initialization counter, to kill simulation if more than two different spacecraft are called
    int scInitCounter;
    // Spacecraft integrator timestep counter
    int integratorCounter;

    int scIDs [2];

    double k; // proportional gain
    double c; // derivative gain

    Eigen::Vector3d Fc_N;
    Eigen::Vector3d L_B1;
    Eigen::Vector3d L_B2;

};


#endif /* CONSTRAINT_DYNAMIC_EFFECTOR_H */
