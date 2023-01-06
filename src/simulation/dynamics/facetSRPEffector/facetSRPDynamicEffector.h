/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef FACET_SRP_DYNAMIC_EFFECTOR_H
#define FACET_SRP_DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SpinningBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spacecraft geometry data */
typedef struct {
  std::vector<double> facetAreas;                       //!< [m^2] vector of facet areas
  std::vector<double> facetSpecCoeffs;                  //!< vector of facet spectral reflection optical coefficients
  std::vector<double> facetDiffCoeffs;                  //!< vector of facet diffuse reflection optical coefficients
  std::vector<Eigen::Vector3d> facetNormals;            //!< vector of facet normals
  std::vector<Eigen::Vector3d> facetLocations;          //!< [m] vector of facet locations
}SpacecraftGeometryData;


/*! @brief faceted atmospheric drag dynamic effector */
class FacetSRPDynamicEffector: public SysModel, public DynamicEffector {
public:
    FacetSRPDynamicEffector();
    ~FacetSRPDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void addFacet(double area, double specCoeff, double diffCoeff, Eigen::Vector3d B_normal_hat, Eigen::Vector3d B_location);

    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;                //!< Sun spice ephemeris input message
    ReadFunctor<SpinningBodyMsgPayload> spinningBodyInMsg1;          //!< Solar array 1 state input message
    ReadFunctor<SpinningBodyMsgPayload> spinningBodyInMsg2;          //!< Solar array 2 state input message

    double theta1;
    double theta2;
    Eigen::Matrix3d dcm1_BF;                                         //!< DCM from F frame to B frame
    Eigen::Matrix3d dcm2_BF;                                         //!< DCM from F frame to B frame

    uint64_t numFacets;                                              //!< Number of facets
    Eigen::Vector3d r_SN_N;                                          //!< [m] Position vector of the Sun with respect to the inertial frame

    StateData *hubPosition;                                          //!< [m/s] Hub inertial velocity vector
    StateData *hubSigma;                                             //!< Hub attitude with respect to the inertial frame represented by MRP
    BSKLogger bskLogger;                                             //!< BSK Logging

private:
    SpacecraftGeometryData scGeometry;                               //!< Struct to hold spacecraft facet data
};

#endif 
