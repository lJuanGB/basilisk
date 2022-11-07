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
#include "architecture/utilities/RigidBodyKinematics.h"
#include <cstring>
#include <iostream>
#include <cmath>

/*! The Constructor.*/
ConstraintDynamicEffector::ConstraintDynamicEffector()
{
    //TODO: Initialize all variables (can be 0 or not, whatever makes sense)
    this->counter = 0.0;
    this->flag = 1;

    // - Initialize constraint dimensions
    this->r_P1B1_B1 = {0.2974;0.9466;0.1246};
    this->r_P2B2_B2 = -{0.2974;0.9466;0.1246};
    this->l = 0.0;
    this->l_P2P1_B1 = {0.7, 0.5, 0.3} / sqrt(pow(0.7,2)+pow(0.5,2)+pow(0.3,2));

    //Do we want to toggle type of constraint here or just always plan on "rigid" version?

    // - Initialize gains
    this->c = pow(this->alpha,2.0);
    this->p = 2 * this->beta;
    this->kI = 0.0;
    this->kI_att = 0.0;
    this->K = 300.0;
    this->P = 30.0;

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
    this->kI = 0.0;
    this->kI_att = 0.0;

    return;
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ConstraintDynamicEffector::linkInStates(DynParamManager& states)
{
    if (this->counter > 1) {
        bskLogger.bskLog(BSK_ERROR, "constraintDynamicEffector: tried to attach more than 2 spacecraft");
    }

    this->hubSigma[counter] = states.getStateObject("hubSigma");
	this->hubOmega[counter] = states.getStateObject("hubOmega");
    this->hubPosition[counter] = states.getStateObject("hubPosition");
    this->hubVelocity[counter] = states.getStateObject("hubVelocity");

    counter++;
    return;
}

/*! This method computes the Forces on Torque on the Spacecraft Body.
 @return void
 @param integTime Integration time
 @param timeStep Current integration time step used
 */
void ConstraintDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
    // TODO: Implement the logic for the constraints
    if (flag == 1) {
        // flag = 1 signifies being called by spacecraft 1

        // - Collect states from both spacecraft
        r_B1N_N = this->hubPosition[0]->getState();
        rDot_B1N_N = this->hubVelocity[0]->getState();
        omega_B1N_B1 = this->hubOmega[0]->getState();
        sig_B1N = this->hubSigma[0]->getState();
        r_B2N_N = this->hubPosition[1]->getState();
        rDot_B2N_N = this->hubVelocity[1]->getState();
        omega_B2N_B2 = this->hubOmega[1]->getState();
        sig_B2N = this->hubOmega[1]->getState();

        // computing direction constraint psi in the N frame
        MRP2C(sig_B1N,dcm_B1N);
        MRP2C(sig_B2N,dcm_B2N);
        m33tMultV3(dcm_B1N,r_P1B1_B1,r_P2B1_N);
        m33tMultV3(dcm_B2N,r_P2B2_B2,r_P2B2_N);
        v3Add(r_P1B1_N,r_B1N_N,r_P1N_N);
        v3Add(r_P2B2_N,r_B2N_N,r_P2N_N);
        v3Subtract(r_P2N_N,r_P1N_N,r_P2P1_N);
        m33tMultV3(dcm_B1N,l_P2P1_B1,l_P2P1_N);
        v3Subtract(r_P2P1_N,l_P2P1_N,psi_N);
        
        // computing length constraint rate of change psiDot in the N frame
        v3Cross(omega_B1N_B1,r_P1B1_B1,rDot_P1B1_B1);
        v3Cross(omega_B2N_B2,r_P2B2_B2,rDot_P2B2_B2);
        m33tMultV3(dcm_B1N,rDot_P1B1_B1,rDot_P1B1_N);
        v3Add(rDot_P1B1_N,rDot_B1N_N,rDot_P1N_N);
        m33tMultV3(dcm_B2N,rDot_P2B2_B2,rDot_P2B2_N);
        v3Add(rDot_P2B2_N,rDot_B2N_N,rDot_P2N_N);
        v3Subtract(rDot_P2N_N,rDot_P1N_N,rDot_P2P1_N);
        m33tMultV3(dcm_B1N,omega_B1N_B1,omega_B1N_N);
        v3Cross(omega_B1N_N,r_P2P1_N,wait_what);
        v3Subtract(rDot_P2P1_N,wait_what,psiDot_N);

        // calculative the difference in angular rate
        m33tMultV3(dcm_B1N,omega_B1N_B1,omega_B1N_N);
        m33MultV3(dcm_B2N,omega_B1N_N,omega_B1N_B2);
        v3Subtract(omega_B2N_B2,omega_B1N_B2,omega_B2B1_B2);

        // calculate the difference in attitude
        subMRP(sig_B1N,sig_B2N,sig_B2B1);

        // computing the constraint force
        v3Scale(this->c,psi_N,F_proportional);
        v3Scale(this->p,psiDot_N,F_derivative);
        v3Add(F_proportional,F_derivative,this->Fc_N);
        m33MultV3(dcm_B1N,Fc_N,Fc_B1);
        this->forceExternal_N = Fc_N;
        this->forceExternal_B = Fc_B1;

        // computing constraint torque from direction constraint
        v3Cross(r_P1B1_B1,Fc_B1,this->torqueExternal_B)

        // computing constraint torque from attitude constraint
        v3Scale(-K,sig_B2B1,L_proportional);
        v3Scale(-P,omega_B2B1_B2,L_derivative);
        v3Add(L_proportional,L_derivative,this->L_B2b);
        
        this->flag = -1;
    } else if (flag == -1) {
        // flag = -1 signifies being called by spacecraft 2

        // computing the constraint force from stored magnitude
        m33MultV3(dcm_B2N,-Fc_N,Fc_B2);
        this->forceExternal_N = -Fc_N;
        this->froceExternal_B = Fc_B2;

        // computing constraint torque
        v3Cross(r_P2B2_B2,Fc_B2,L_B2a)
        L_B2b = K*-sig_B2B1
        v3Add(this->torqueExternal_B)

        this->flag = 1;
    } else {
        bskLogger.bskLog(BSK_ERROR, "constraintDynamicEffector: tried to apply dynamics to a nonexistant spacecraft");
    }
    
    return;
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
    // TODO: Compute integral feedback term

}
