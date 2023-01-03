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


/* modify the path to reflect the new module names */
#include "platformRotation.h"
#include "string.h"
#include <math.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_platformRotation(platformRotationConfig *configData, int64_t moduleID)
{
    SpinningBodyMsg_C_init(&configData->SpinningBodyRef1OutMsg);
    SpinningBodyMsg_C_init(&configData->SpinningBodyRef2OutMsg);
    BodyHeadingMsg_C_init(&configData->bodyHeadingOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_platformRotation(platformRotationConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input messages are included
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: platformRotation.vehConfigInMsg wasn't connected.");
    }

    if (CmdTorqueBodyMsg_C_isLinked(&configData->deltaHInMsg)) {
        configData->momentumDumping = 1;
    }
    else {
        configData->momentumDumping = 0;
    }
}

/*! This method updates the platformAngles message based on the updated information about the system center of mass
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_platformRotation(platformRotationConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create message buffers */
    VehicleConfigMsgPayload  vehConfigMsgIn;
    CmdTorqueBodyMsgPayload  deltaHMsgIn;
    SpinningBodyMsgPayload   spinningBodyRef1Out;
    SpinningBodyMsgPayload   spinningBodyRef2Out;
    BodyHeadingMsgPayload    bodyHeadingOut;

    /*! - zero the output messages */
    spinningBodyRef1Out = SpinningBodyMsg_C_zeroMsgPayload();
    spinningBodyRef2Out = SpinningBodyMsg_C_zeroMsgPayload();
    bodyHeadingOut = BodyHeadingMsg_C_zeroMsgPayload();

    /*! read the attitude navigation message */
    vehConfigMsgIn = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! read the delta H message */
    double offset[3] = {0};
    if (configData->momentumDumping == 1) {
        deltaHMsgIn = CmdTorqueBodyMsg_C_read(&configData->deltaHInMsg);
        if (v3Norm(deltaHMsgIn.torqueRequestBody) > EPS) {
            v3Copy(deltaHMsgIn.torqueRequestBody, offset);
        }
    }

    /*! compute CM position w.r.t. M frame origin, in M coordinates */
    double r_CM_M[3], r_CM_F[3], r_CB_B[3], r_CB_M[3], MB[3][3];
    MRP2C(configData->sigma_MB, MB);
    v3Copy(vehConfigMsgIn.CoM_B, r_CB_B);
    m33MultV3(MB, r_CB_B, r_CB_M);
    v3Add(r_CB_M, configData->r_BM_M, r_CM_M);

    double FM[3][3];
    computeFinalRotation(r_CM_M, configData->r_FM_F, configData->r_TF_F, configData->T_F, FM);

    /*! extract theta1 and theta2 angles */
    spinningBodyRef1Out.theta = atan2(FM[1][2], FM[1][1]);
    spinningBodyRef1Out.thetaDot = 0;
    spinningBodyRef2Out.theta = atan2(FM[2][0], FM[0][0]);
    spinningBodyRef2Out.thetaDot = 0;

    printf("theta1 = %4.f, theta2 = %4.f \n", FM[1][2], FM[1][1]);

    /* write output spinning body messages */
    SpinningBodyMsg_C_write(&spinningBodyRef1Out, &configData->SpinningBodyRef1OutMsg, moduleID, callTime);
    SpinningBodyMsg_C_write(&spinningBodyRef2Out, &configData->SpinningBodyRef2OutMsg, moduleID, callTime);

    /*! define mapping between final platform frame and body frame F3B */
    double FB[3][3];
    m33MultM33(FM, MB, FB);

    /*! compute thruster direction in body frame coordinates */
    m33tMultV3(FB, configData->T_F, bodyHeadingOut.rHat_XB_B);
    v3Normalize(bodyHeadingOut.rHat_XB_B, bodyHeadingOut.rHat_XB_B);

    /* write output message */
    BodyHeadingMsg_C_write(&bodyHeadingOut, &configData->bodyHeadingOutMsg, moduleID, callTime);

    return;
}


double computeSecondRotation(double r_CM_F[3], double r_FM_F[3], double r_TF_F[3], double r_CT_F[3], double T_F_hat[3])
{
    double a, b, c1, c2, aVec[3], bVec[3];

    v3Add(r_FM_F, r_TF_F, aVec);
    a = v3Norm(aVec);

    if (a < EPS) {
        return 0.0;
    }

    v3Copy(r_CM_F, bVec);
    b = v3Norm(bVec);
    c1 = v3Norm(r_CT_F);

    double beta, nu;
    beta = acos( -fmin( fmax( v3Dot(aVec, T_F_hat) / a, -1 ), 1 ) );
    nu = acos( -fmin( fmax( v3Dot(aVec, r_CT_F) / (a*c1), -1 ), 1 ) );

    c2 = a*cos(beta) + sqrt(b*b - a*a*sin(beta)*sin(beta));

    double cosGamma1, cosGamma2;
    cosGamma1 = (a*a + b*b - c1*c1) / (2*a*b);
    cosGamma2 = (a*a + b*b - c2*c2) / (2*a*b);

    double psi = asin( fmin( fmax( (c1*sin(nu)*cosGamma2 - c2*sin(beta)*cosGamma1)/b, -1 ), 1 ) );

    return psi;
}

double computeThirdRotation(double e_theta[3], double F2M[3][3])
{
    double e1, e2, e3;
    e1 = e_theta[0];  e2 = e_theta[1];  e3 = e_theta[2];

    double A, B, C, Delta;
    A = 2 * (F2M[1][0]*e2*e2 + F2M[0][0]*e1*e2 + F2M[2][0]*e2*e3) - F2M[1][0];
    B = 2 * (F2M[2][0]*e1 - F2M[0][0]*e3);
    C = F2M[1][0];
    Delta = B*B - 4*A*C;

    /* compute exact solution or best solution depending on Delta */
    double t, t1, t2, y, y1, y2, theta;
    if (fabs(A) < EPS) {
        if (fabs(B) < EPS) {
            if (C > 0) {
                theta = MPI;
            }
            else {
                theta = 0.0;
            }
        }
        else {
            t = - C / B;
            theta = 2*atan(t);
        }
    }
    else {
        if (Delta >= 0) {
            t1 = (-B + sqrt(Delta)) / (2*A);
            t2 = (-B - sqrt(Delta)) / (2*A);
            t = t1;
            if (fabs(t2) < fabs(t1)) {
                t = t2;
            }
            theta = 2*atan(t);
        }
        else {
            if (fabs(B) < EPS) {
                t = 0.0;
            }
            else {
                t1 = (A-C + sqrt((A-C)*(A-C) + B*B)) / B;
                t2 = (A-C - sqrt((A-C)*(A-C) + B*B)) / B;
                y1 = (A*t1*t1 + B*t1 + C) / (1 + t1*t1);
                y2 = (A*t2*t2 + B*t2 + C) / (1 + t2*t2);

                t = t1;
                // if (fabs(y2) < fabs(y1)) {
                //     t = t2;
                // }
            }
            theta = 2*atan(t);
            y = (A*t*t + B*t + C) / (1 + t*t);
            if (fabs(A) < fabs(y)) {
                theta = MPI;
            }
        }
    }

    return theta;
}

void computeFinalRotation(double r_CM_M[3], double r_FM_F[3], double r_TF_F[3], double T_F[3], double FM[3][3])
{
    /*! define unit vectors of CM direction in M coordinates and thrust direction in F coordinates */
    double r_CM_F[3], r_CM_M_hat[3], r_CM_F_hat[3], T_F_hat[3];
    v3Normalize(r_CM_M, r_CM_M_hat);
    v3Normalize(T_F, T_F_hat);
    v3Copy(r_CM_M_hat, r_CM_F_hat);        // assume zero initial rotation between F and M

    /*! compute first rotation to make T_F parallel to r_CM */
    double phi, e_phi[3];
    phi = acos( fmin( fmax( v3Dot(T_F_hat, r_CM_F_hat), -1 ), 1 ) );
    v3Cross(T_F_hat, r_CM_F_hat, e_phi);
    // If phi = PI, e_phi can be any vector perpendicular to F_current_B
    if (fabs(phi-MPI) < EPS) {
        phi = MPI;
        if (fabs(T_F_hat[0]) > EPS) {
            e_phi[0] = -(T_F_hat[1]+T_F_hat[2]) / T_F_hat[0];
            e_phi[1] = 1;
            e_phi[2] = 1;
        }
        else if (fabs(T_F_hat[1]) > EPS) {
            e_phi[0] = 1;
            e_phi[1] = -(T_F_hat[0]+T_F_hat[2]) / T_F_hat[1];
            e_phi[2] = 1;
        }
        else {
            e_phi[0] = 1;
            e_phi[1] = 1;
            e_phi[2] = -(T_F_hat[0]+T_F_hat[1]) / T_F_hat[2];
        }
    }
    else if (fabs(phi) < EPS) {
        phi = 0;
    }
    // normalize e_phi
    v3Normalize(e_phi, e_phi);

    /*! define intermediate platform rotation F1M */
    double F1M[3][3], PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, F1M);

    /*! rotate r_CM_F */
    m33MultV3(F1M, r_CM_M, r_CM_F);

    /*! compute position of CM w.r.t. thrust application point T */
    double r_CT_F[3], r_CT_F_hat[3];
    v3Subtract(r_CM_F, r_FM_F, r_CT_F);
    v3Subtract(r_CT_F, r_TF_F, r_CT_F);
    v3Normalize(r_CT_F, r_CT_F_hat);

    /*! compute second rotation to zero the offset between T_F and r_CT_F */
    double psi, e_psi[3];
    v3Cross(T_F_hat, r_CT_F_hat, e_psi);
    v3Normalize(e_psi, e_psi);
    psi = computeSecondRotation(r_CM_F, r_FM_F, r_TF_F, r_CT_F, T_F_hat);

    /*! define intermediate platform rotation F2M */
    double F2F1[3][3], F2M[3][3], PRV_psi[3];
    v3Scale(psi, e_psi, PRV_psi);
    PRV2C(PRV_psi, F2F1);
    m33MultM33(F2F1, F1M, F2M);

    /*! compute third rotation to make the frame compliant with the platform constraint */
    double theta, e_theta[3];
    m33MultV3(F2M, r_CM_M, e_theta);
    v3Normalize(e_theta, e_theta);
    theta = computeThirdRotation(e_theta, F2M);

    /*! define final platform rotation F3M */
    double F3F2[3][3], PRV_theta[3];
    v3Scale(theta, e_theta, PRV_theta);
    PRV2C(PRV_theta, F3F2);
    m33MultM33(F3F2, F2M, FM);

}
