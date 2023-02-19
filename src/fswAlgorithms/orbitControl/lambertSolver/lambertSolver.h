/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef LAMBERTSOLVER_H
#define LAMBERTSOLVER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>

/*! @brief This module is provides a Lyapunov feedback control law for waypoint to waypoint guidance and control about
 * a small body. The waypoints are defined in the Hill frame of the body.
 */
class LambertSolver: public SysModel {
public:
    LambertSolver();
    ~LambertSolver();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    
private:
    //    void readMessages();
    void writeMessages(uint64_t CurrentSimNanos);
    void problemGeometry();
    void findx();
    Eigen::MatrixXd computeVelocities(double x);
    std::vector<double> getInitialGuess(double lambda, double T);
    double x2tof(double x, int N, double lambda);
    std::vector<double> dTdx(double x, double T, double lambda);
    
    double householder(double T, double x0, int N, double tol, int iter_max);
    double halley(double T, double x0, int N, double tol, int iter_max);
    double getTmin(double T0M, double x0, int N, double tol, int iter_max);
    double hypergeometricF(double a, double b, double c, double z, double tol);

public:
//    ReadFunctor<NavTransMsgPayload> navTransInMsg;  //!< translational navigation input message
    
    std::vector<Message<LambertSolutionMsgPayload>*>  lambertSolutionOutMsgs;    //!< lambert solution output messages

    BSKLogger bskLogger;              //!< -- BSK Logging
    
    std::string solverName; //!< name of lambert algorithm
    Eigen::Vector3d r1vec;
    Eigen::Vector3d r2vec;
    double transferTime;
    double mu;
    int M; //!< number of revolutions

private:
//    NavTransMsgPayload navTransInMsgBuffer;  //!< local copy of message buffer
    double T;
    double lambda;
    double r1;
    double r2;
    double c;
    double s;
    double x;
    double x_2;
    bool noMultiRevSolution;
    Eigen::Vector3d v1vec;
    Eigen::Vector3d v2vec;
    Eigen::Vector3d v1vec_2;
    Eigen::Vector3d v2vec_2;
    Eigen::Vector3d i_r1;
    Eigen::Vector3d i_t1;
    Eigen::Vector3d i_r2;
    Eigen::Vector3d i_t2;
    Eigen::Vector3d i_h;

};


#endif
