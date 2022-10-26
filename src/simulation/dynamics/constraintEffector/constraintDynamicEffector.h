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
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    

public:
    // Gains for PD controller
    double alpha;
    double beta;
    double kI;
    Eigen::Matrix3d K;
    Eigen::Matrix3d P;

	std::vector<StateData*> hubSigma;                           //!< class variable
    std::vector<StateData*> hubOmega;                           //!< class varaible
    std::vector<StateData*> hubPosition;                           //!< class variable
    std::vector<StateData*> hubVelocity;                           //!< class varaible
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    double counter;

    double c;
    double p;

};


#endif /* CONSTRAINT_DYNAMIC_EFFECTOR_H */
