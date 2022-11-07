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
#include <cstring>
#include <iostream>
#include <cmath>

/*! The Constructor.*/
ConstraintDynamicEffector::ConstraintDynamicEffector()
{
    //TODO: Initialize all variables (can be 0 or not, whatever makes sense)
    counter = 0;

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
    // TODO: Reset the integral value

    return;
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ConstraintDynamicEffector::linkInStates(DynParamManager& states)
{
    // TODO: yell scream and die if counter > 1: bskLogger.bskLog(BSK_ERROR, "constraintDynamicEffector: tried to attach more than 2 spacecraft");
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
    // Populate this->forceExternal_N (or this->forceExternal_B) and this->forceExternal_B
    // Have a flag set to 1 or -1 to know which spacecraft you're working with with if else statements
    // To grab sigma_B1N, do this: sigma_B1N = this->hubSigma[0]->getState()
    
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
