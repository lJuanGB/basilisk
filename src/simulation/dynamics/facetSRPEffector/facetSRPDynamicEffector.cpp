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

#include <iostream>
#include "facetSRPDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <cmath>

/*! The constructor */
FacetSRPDynamicEffector::FacetSRPDynamicEffector()
{
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
	this->numFacets = 0;
	this->theta1 = 0.0;
	this->theta2 = 0.0;

	return;
}

/*! The destructor */
FacetSRPDynamicEffector::~FacetSRPDynamicEffector()
{
	return;
}

/*! The reset member function. This method checks to ensure the input message is linked.
@return void
*/
void FacetSRPDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
	// Check if input messages have not been included
	if (!this->sunInMsg.isLinked()) {
		bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.sunInMsg was not linked.");
	}

	if (!this->spinningBodyInMsg1.isLinked()) {
		bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.spinningBodyInMsg1 was not linked.");
	}

	if (!this->spinningBodyInMsg2.isLinked()) {
		bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.spinningBodyInMsg2 was not linked.");
	}

    return;
}

/*! The SRP dynamic effector does not write any output messages
@return void
 */
void FacetSRPDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}

/*! This member function populates the spacecraft geometry structure with user-input facet information
    @param area
    @param specCoeff
    @param diffCoeff
    @param normal_hat
    @param location
 */
void FacetSRPDynamicEffector::addFacet(double area, double specCoeff, double diffCoeff, Eigen::Vector3d normal_hat, Eigen::Vector3d location)
{
	this->scGeometry.facetAreas.push_back(area);
	this->scGeometry.facetSpecCoeffs.push_back(specCoeff);
    this->scGeometry.facetDiffCoeffs.push_back(diffCoeff);
	this->scGeometry.facetNormals.push_back(normal_hat);
	this->scGeometry.facetLocations.push_back(location);
	this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the faceted SRP effector to the hub attitude and position,
which are required for calculating SRP forces and torques.
 @return void
 @param states dynamic parameter states
 */

void FacetSRPDynamicEffector::linkInStates(DynParamManager& states)
{
	this->hubSigma = states.getStateObject("hubSigma");
	this->hubPosition = states.getStateObject("hubPosition");
}

/*! This method computes the body forces and torques for the SRP effector.
*/
void FacetSRPDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
    /*! Read the input messages */
    SpicePlanetStateMsgPayload sunMsgBuffer;
    sunMsgBuffer = sunInMsg.zeroMsgPayload;
    sunMsgBuffer = this->sunInMsg();
    this->r_SN_N = cArray2EigenVector3d(sunMsgBuffer.PositionVector);

    if (this->spinningBodyInMsg1.isLinked() && this->spinningBodyInMsg1.isWritten())
    {
        SpinningBodyMsgPayload incomingSpinningBodyMsg1;
        incomingSpinningBodyMsg1 = spinningBodyInMsg1.zeroMsgPayload;
        incomingSpinningBodyMsg1 = this->spinningBodyInMsg1();
        this->theta1 = incomingSpinningBodyMsg1.theta;
    }

    if (this->spinningBodyInMsg2.isLinked() && this->spinningBodyInMsg2.isWritten())
    {
        SpinningBodyMsgPayload incomingSpinningBodyMsg2;
        incomingSpinningBodyMsg2 = spinningBodyInMsg2.zeroMsgPayload;
        incomingSpinningBodyMsg2 = this->spinningBodyInMsg2();
        this->theta2 = incomingSpinningBodyMsg2.theta;
    }

    /*! Determine solar array DCMs, dcm1_BF and dcm2_BF */
    double prv1[3] = {-this->theta1, 0, 0};
    double prv2[3] = {this->theta2, 0, 0};

    double dcm1_BF[3][3];
    double dcm2_BF[3][3];
    PRV2C(prv1, dcm1_BF);
    PRV2C(prv2, dcm2_BF);

    this->dcm1_BF = c2DArray2EigenMatrix3d(dcm1_BF);
    this->dcm2_BF = c2DArray2EigenMatrix3d(dcm2_BF);

    double speedLight = 299792458.0;
    double AstU = 149597870700.0;
    double solarRadFlux = 1368.0;

    //! Convert hub MRP of B frame wrt inertial frame to a DCM
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

    //! Get the hub position with respect to the inertial frame
    Eigen::Vector3d r_BN_N;
    r_BN_N = this->hubPosition->getState();

    //! Calculate the unit vector in the direction of the sun
    Eigen::Vector3d r_SB_B = dcm_BN * (this->r_SN_N - r_BN_N);

    Eigen::Vector3d facetSRPForce, facetSRPTorque;
	Eigen::Vector3d totalSRPForce, totalSRPTorque;

	//! Zero out the structure force/torque for the drag set
    double projectedArea = 0.0;
    double projectionTerm = 0.0;
    facetSRPForce.setZero();
    facetSRPTorque.setZero();
    totalSRPForce.setZero();
    totalSRPTorque.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
    //! Calculate the SRP pressure acting on the spacecraft center of mass
    double numAU = AstU / r_SB_B.norm();
    double SRPPressure = (solarRadFlux / speedLight) * numAU * numAU;

    //! Calculate the unit vector pointing from the spacecraft to the Sun
    Eigen::Vector3d sHat;
    sHat = r_SB_B / r_SB_B.norm();

	for(int i = 0; i < this->numFacets; i++){

		if (i == 6 || i == 7)
		{
            this->scGeometry.facetNormals[i] = this->dcm1_BF * this->scGeometry.facetNormals[i];
		}
		if (i == 8 || i == 9)
		{
		    this->scGeometry.facetNormals[i] = this->dcm2_BF * this->scGeometry.facetNormals[i];
		}

		projectionTerm = this->scGeometry.facetNormals[i].dot(sHat);
		projectedArea = this->scGeometry.facetAreas[i] * projectionTerm;

		if(projectedArea > 0.0){

		    //! Calculate the incidence angle between the facet normal vector and the Sun-direction vector
			double cosTheta = projectionTerm;
            Eigen::Vector3d intermediate;
            intermediate = sHat.cross(this->scGeometry.facetNormals[i]);
            double sinTheta = intermediate.norm();
			double theta = atan2(sinTheta, cosTheta);

            //! Compute the SRP force acting on the ith facet
            facetSRPForce = -SRPPressure * projectedArea * cos(theta) * ( (1-this->scGeometry.facetSpecCoeffs[i]) * sHat + 2 * ( (this->scGeometry.facetDiffCoeffs[i] / 3) + this->scGeometry.facetSpecCoeffs[i] * cos(theta)) * this->scGeometry.facetNormals[i] );

            //! Compute the SRP torque acting on the ith facet
            facetSRPTorque = this->scGeometry.facetLocations[i].cross(facetSRPForce);

            //! Compute the total SRP force and torque acting on the spacecraft
            totalSRPForce = totalSRPForce + facetSRPForce;
            totalSRPTorque = totalSRPTorque + facetSRPTorque;
		}
	}

	//! Write the total SRP force and torque local variables to the dynamic effector variables
	this->forceExternal_B = totalSRPForce;
	this->torqueExternalPntB_B = totalSRPTorque;

    return;
}

/*! This method is called to update the current position of the Sun with respect to the inertial frame
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void FacetSRPDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    
	return;
}
