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

#include "thrusterStateEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! The Constructor.*/
ThrusterStateEffector::ThrusterStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // initialize internal variables
    this->effectorID++;

    // TODO: intialize variables here (set some to 0, others to identity, whatever)

    return;
}

uint64_t ThrusterStateEffector::effectorID = 1;

/*! The destructor. */
ThrusterStateEffector::~ThrusterStateEffector()
{
    this->effectorID = 1;    /* reset the panel ID*/

    return;
}

/*! This method is used to reset the module.
 @return void
 */
void ThrusterStateEffector::Reset(uint64_t CurrentSimNanos)
{
    // TODO: set the integrator state to 0

    return;
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ThrusterStateEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubOmega = states.getStateObject("hubOmega");
}

/*! This method allows the thruster state effector to register its state kappa with the dyn param manager */
void ThrusterStateEffector::registerStates(DynParamManager& states)
{

    // TODO: figure out integrator states

    return;
}

/*! This method is used to find the derivatives for the thruster stateEffector */
void ThrusterStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{

    // TODO: figure out integrator states
   
    return;
}

void ThrusterStateEffector::updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // TODO: compute the forces and torques with the PDI controller
    // TODO: figure out how to affect both spacecraft

}


/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterStateEffector::UpdateState(uint64_t CurrentSimNanos)
{

}
