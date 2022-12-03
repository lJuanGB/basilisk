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

#include <cstring>
#include <iostream>
#include <cmath>

#include "constraintDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cstring>
#include <iostream>
#include <cmath>

/*! The Constructor.*/
ConstraintDynamicEffector::ConstraintDynamicEffector()
{ 
    // counters and flags
    this->scInitCounter = 0;
    this->scCounterFlag = 1;
    this->integratorCounter = 1;

    // - Initialize constraint dimensions
    this->r_P1B1_B1.setZero();
    this->r_P2B2_B2.setZero();
    this->r_P2P1_B1Init.setZero();

    // - Initialize gains
    this->alpha = 0.0;
    this->beta = 0.0;
    this->k = 0.0;
    this->c = 0.0;
    this->kI = -1;
    this->kI_att = -1;
    this->K = -1;
    this->P = -1;

    this->Fc_N.setZero();
    this->L_B1.setZero();
    this->L_B2.setZero();

    return;
}

/*! The destructor. */
ConstraintDynamicEffector::~ConstraintDynamicEffector()
{
    return;
}


/*! This method is used to reset the module.
 @return void
 */
void ConstraintDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    this->k = pow(this->alpha, 2);
    this->c = 2 * this->beta;
    this->scCounterFlag = 1;
    this->integratorCounter = 1;

    return;
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ConstraintDynamicEffector::linkInStates(DynParamManager& states, uint64_t spacecraftID)
{
    if (this->scInitCounter > 1) {
        bskLogger.bskLog(BSK_ERROR, "constraintDynamicEffector: tried to attach more than 2 spacecraft");
    }

    this->scIDs[scInitCounter] = spacecraftID;

    this->hubSigma.push_back(states.getStateObject("hubSigma"));
	this->hubOmega.push_back(states.getStateObject("hubOmega"));
    this->hubPosition.push_back(states.getStateObject("hubPosition"));
    this->hubVelocity.push_back(states.getStateObject("hubVelocity"));

    this->scInitCounter++;
    return;
}

/*! This method computes the Forces on Torque on the Spacecraft Body.
 @return void
 @param integTime Integration time
 @param timeStep Current integration time step used
 */
void ConstraintDynamicEffector::computeForceTorque(double integTime, double timeStep, uint64_t spacecraftID)
{
    if (this->scInitCounter == 2) {
        // assigning the constraint force and torque from stored values
        if (spacecraftID == scIDs[0]) {
            this->forceExternal_N = this->Fc_N;
            this->torqueExternalPntB_B = this->L_B1;
        }
        else if (spacecraftID == scIDs[1]) {
            this->forceExternal_N = - this->Fc_N;
            this->torqueExternalPntB_B = this->L_B2;
        }
        return;
    }
}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ConstraintDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    if (this->scInitCounter == 2)
    {
        // - Collect states from both spacecraft
        Eigen::Vector3d r_B1N_N = this->hubPosition[0]->getState();
        Eigen::Vector3d rDot_B1N_N = this->hubVelocity[0]->getState();
        Eigen::Vector3d omega_B1N_B1 = this->hubOmega[0]->getState();
        Eigen::MRPd sigma_B1N;
        sigma_B1N = (Eigen::Vector3d)this->hubSigma[0]->getState();
        Eigen::Vector3d r_B2N_N = this->hubPosition[1]->getState();
        Eigen::Vector3d rDot_B2N_N = this->hubVelocity[1]->getState();
        Eigen::Vector3d omega_B2N_B2 = this->hubOmega[1]->getState();
        Eigen::MRPd sigma_B2N;
        sigma_B2N = (Eigen::Vector3d)this->hubOmega[1]->getState();

        // computing direction constraint psi in the N frame
        Eigen::Matrix3d dcm_B1N = (sigma_B1N.toRotationMatrix()).transpose();
        Eigen::Matrix3d dcm_B2N = (sigma_B2N.toRotationMatrix()).transpose();
        Eigen::Vector3d r_P1B1_N = dcm_B1N.transpose() * this->r_P1B1_B1;
        Eigen::Vector3d r_P2B2_N = dcm_B2N.transpose() * this->r_P2B2_B2;
        Eigen::Vector3d r_P2P1_N = r_P2B2_N + r_B2N_N - r_P1B1_N - r_B1N_N;

        // computing length constraint rate of change psiPrime in the N frame
        Eigen::Vector3d rDot_P1B1_B1 = omega_B1N_B1.cross(this->r_P1B1_B1);
        Eigen::Vector3d rDot_P2B2_B2 = omega_B2N_B2.cross(this->r_P2B2_B2);
        Eigen::Vector3d rDot_P1N_N = dcm_B1N.transpose() * rDot_P1B1_B1 + rDot_B1N_N;
        Eigen::Vector3d rDot_P2N_N = dcm_B2N.transpose() * rDot_P2B2_B2 + rDot_B2N_N;
        Eigen::Vector3d rDot_P2P1_N = rDot_P2N_N - rDot_P1N_N;
        Eigen::Vector3d omega_B1N_N = dcm_B1N.transpose() * omega_B1N_B1;

        // define the constraints
        this->psi_N = r_P2P1_N - dcm_B1N.transpose() * this->r_P2P1_B1Init;
        this->psi_B1 = dcm_B1N * this->psi_N;
        this->psiPrime_N = rDot_P2P1_N - omega_B1N_N.cross(r_P2P1_N);
        this->psiPrime_B1 = dcm_B1N * this->psiPrime_N;

        // calculate the difference in angular rate
        Eigen::Vector3d omega_B1N_B2 = dcm_B2N * dcm_B1N.transpose() * omega_B1N_B1;
        omega_B2B1_B2 = omega_B2N_B2 - omega_B1N_B2;

        // calculate the difference in attitude
        sigma_B2B1 = eigenMRPd2Vector3d(eigenC2MRP(dcm_B2N * dcm_B1N.transpose()));

        // computing the constraint force
        this->Fc_N = this->k * this->psi_N + this->c * this->psiPrime_N;
        this->forceExternal_N = this->Fc_N;

        // computing constraint torque for spacecraft 1
        Eigen::Vector3d Fc_B1 = dcm_B1N * this->Fc_N;
        this->L_B1 = (dcm_B1N * r_P2P1_N + this->r_P1B1_B1).cross(Fc_B1);

        // computing constraint torque for spacecraft 2
        Eigen::Vector3d Fc_B2 = dcm_B2N * this->Fc_N;
        Eigen::Vector3d L_B2_len = - this->r_P2B2_B2.cross(Fc_B2);
        
        // total torque imparted on spacecraft 2
        this->L_B2 = L_B2_len;
        if (this->K > 0 && this->P > 0)
        {
            Eigen::Vector3d L_B2_att = -this->K * sigma_B2B1 - this->P * omega_B2B1_B2;
            this->L_B2 += L_B2_att;
        }
    }
}
