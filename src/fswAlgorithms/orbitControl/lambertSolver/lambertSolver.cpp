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

#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>
#include <math.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSolver::LambertSolver()
{
    /* create output message objects */
    Message<LambertSolutionMsgPayload> *msgLambert1;
    msgLambert1 = new Message<LambertSolutionMsgPayload>;
    this->lambertSolutionOutMsgs.push_back(msgLambert1);
    Message<LambertSolutionMsgPayload> *msgLambert2;
    msgLambert2 = new Message<LambertSolutionMsgPayload>;
    this->lambertSolutionOutMsgs.push_back(msgLambert2);
}

/*! Module Destructor */
LambertSolver::~LambertSolver()
{
    /* free up output message objects */
    for (long unsigned int c=0; c<this->lambertSolutionOutMsgs.size(); c++) {
        delete this->lambertSolutionOutMsgs.at(c);
    }
}

/*! This method is used to reset the module and checks that required input messages are connected.
    @return void
*/
void LambertSolver::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
//    if (!this->navTransInMsg.isLinked()) {
//        bskLogger.bskLog(BSK_ERROR, "lambertSolver.navTransInMsg was not linked.");
//    }
    // check if input parameters are valid
    if (this->mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.mu must be positive.");
    }
    if (this->transferTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.transferTime must be positive.");
    }
    if (this->M < 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.M must be zero or positive.");
    }
}

/*! This is the main method that gets called every time the module is updated. It computes the solution of Lambert's problem.
    @return void
*/
void LambertSolver::UpdateState(uint64_t CurrentSimNanos)
{
//    this->readMessages();
    
    this->problemGeometry();
    this->findx();
    
    if (this->noMultiRevSolution){
        this->x = NAN;
        this->v1vec = {NAN, NAN, NAN};
        this->v2vec = {NAN, NAN, NAN};
        this->x_sol2 = NAN;
        this->v1vec_sol2 = {NAN, NAN, NAN};
        this->v2vec_sol2 = {NAN, NAN, NAN};
        
        return;
    }
    
    Eigen::MatrixXd vvecs = this->computeVelocities(this->x);
    this->v1vec = vvecs.col(0);
    this->v2vec = vvecs.col(1);
    if (M > 0){
        Eigen::MatrixXd vvecs_sol2 = this->computeVelocities(this->x_sol2);
        this->v1vec_sol2 = vvecs_sol2.col(0);
        this->v2vec_sol2 = vvecs_sol2.col(1);
    }
    else{
        this->x_sol2 = NAN;
        this->v1vec_sol2 = {NAN, NAN, NAN};
        this->v2vec_sol2 = {NAN, NAN, NAN};
    }
    
    this->writeMessages(CurrentSimNanos);
    
    std::cout << std::endl;
    std::cout << "x: " << this->x << ", " << this->x_sol2 << std::endl;
    std::cout << "v1: " << this->v1vec[0] << ", " << this->v1vec[1] << ", " << this->v1vec[2] << std::endl;
    std::cout << "v2: " << this->v2vec[0] << ", " << this->v2vec[1] << ", " << this->v2vec[2] << std::endl;
    std::cout << "v1_2: " << this->v1vec_sol2[0] << ", " << this->v1vec_sol2[1] << ", " << this->v1vec_sol2[2] << std::endl;
    
    std::cout << "v2_2: " << this->v2vec_sol2[0] << ", " << this->v2vec_sol2[1] << ", " << this->v2vec_sol2[2] << std::endl;
    
}

/*! This method reads the input messages each call of updateState
    @return void
*/
//void LambertSolver::readMessages(){
//    /* read in the input messages */
//    navTransInMsgBuffer = this->navTransInMsg();
//}

/*! This method reads the input messages each call of updateState
    @return void
*/
void LambertSolver::writeMessages(uint64_t CurrentSimNanos){
    // Always zero the output message buffers before assigning values
    LambertSolutionMsgPayload lambertSolutionOutMsgBuffer1;
    
    lambertSolutionOutMsgBuffer1 = this->lambertSolutionOutMsgs.at(0)->zeroMsgPayload;
    eigenVector3d2CArray(this->v1vec, lambertSolutionOutMsgBuffer1.v1);
    eigenVector3d2CArray(this->v2vec, lambertSolutionOutMsgBuffer1.v2);
    lambertSolutionOutMsgBuffer1.x = this->x;
    
    // Write to the output message
    this->lambertSolutionOutMsgs.at(0)->write(&lambertSolutionOutMsgBuffer1, this->moduleID, CurrentSimNanos);
    
    LambertSolutionMsgPayload lambertSolutionOutMsgBuffer2;
    
    lambertSolutionOutMsgBuffer2 = this->lambertSolutionOutMsgs.at(1)->zeroMsgPayload;
    eigenVector3d2CArray(this->v1vec_sol2, lambertSolutionOutMsgBuffer2.v1);
    eigenVector3d2CArray(this->v2vec_sol2, lambertSolutionOutMsgBuffer2.v2);
    lambertSolutionOutMsgBuffer2.x = this->x_sol2;
    
    // Write to the output message
    this->lambertSolutionOutMsgs.at(1)->write(&lambertSolutionOutMsgBuffer2, this->moduleID, CurrentSimNanos);
}

void LambertSolver::problemGeometry()
{
    // chord vector
    Eigen::Vector3d cvec = this->r2vec - this->r1vec;
    // compute magnitudes
    this->c = cvec.norm();
    this->r1 = r1vec.norm();
    this->r2 = r2vec.norm();
    // semiperimeter
    this->s = 1.0/2.0*(this->r1+this->r2+this->c);
    
    // compute orbit frame unit vectors
    this->i_r1 = this->r1vec/this->r1;
    this->i_r2 = this->r2vec/this->r2;
    // check if the two position vectors define a plane
    if (this->i_r1.cross(this->i_r2).norm() < 1e-8){
        bskLogger.bskLog(BSK_WARNING, "lambertSolver: position vectors r1 and r2 don't define a plane .");
    }
    this->i_h = this->i_r1.cross(this->i_r2);///(this->i_r1.cross(this->i_r2).norm());
    
    if (this->i_h(2) < 0.0){
        this->lambda = -this->lambda;
        this->i_t1 = this->i_r1.cross(this->i_h);
        this->i_t2 = this->i_r2.cross(this->i_h);
    }
    else{
        this->i_t1 = this->i_h.cross(this->i_r1);
        this->i_t2 = this->i_h.cross(this->i_r2);
    }
    
    // lambda parameter
    this->lambda = sqrt(1.0-this->c/this->s);
    // non-dimensional time-of-flight
    this->T = sqrt(2.0*this->mu/pow(this->s,3))*this->transferTime;
}

void LambertSolver::findx()
{
    // get initial guess for iteration variable x
    std::vector<double> x0 = getInitialGuess(this->lambda, this->T);
    
    if (this->noMultiRevSolution){
        return;
    }
    
    // find x that satisfies time-of-flight constraint using numerical root finder
    if (this->solverName == "Gooding"){
        // for Gooding algorithm use halley root finder
        this->x = halley(this->T, x0[0], this->M, 1e-5, 12);
        if (this->M > 0){
            this->x_sol2 = halley(this->T, x0[1], this->M, 1e-5, 12);
        }
    }
    else if (this->solverName == "Izzo"){
        // for Izzo algorithm use 3rd order householder root finder
        this->x = householder(this->T, x0[0], this->M, 1e-5, 12);
        if (this->M > 0){
            this->x_sol2 = householder(this->T, x0[1], this->M, 1e-5, 12);
        }
    }
}

Eigen::MatrixXd LambertSolver::computeVelocities(double x)
{
    double y = sqrt(1.0-pow(this->lambda,2)*(1.0-pow(x,2)));
    double gamma = sqrt(this->mu * this->s/2);
    double rho = (this->r1 - this->r2)/(this->c);
    double sigma = sqrt(1.0-pow(rho,2));
    
    // compute velocity components for v1vec and v2vec
    double Vr1 = gamma*((this->lambda*y - x) - rho*(this->lambda*y+x))/(this->r1);
    double Vr2 = -gamma*((this->lambda*y - x) + rho*(this->lambda*y+x))/(this->r2);
    double Vt1 = gamma*sigma*(y+this->lambda*x)/(this->r1);
    double Vt2 = gamma*sigma*(y+this->lambda*x)/(this->r2);
    
    // initial and final velocity of Lambert solution orbit
    Eigen::Vector3d v1 = Vr1*this->i_r1 + Vt1*this->i_t1;
    Eigen::Vector3d v2 = Vr2*this->i_r2 + Vt2*this->i_t2;
    
    Eigen::MatrixXd vvecs(3,2);
    vvecs.col(0) = v1;
    vvecs.col(1) = v2;
    
    return vvecs;
}

std::vector<double> LambertSolver::getInitialGuess(double lambda, double T)
{
    // T for x=0 and zero revolution
    double T00 = acos(lambda) + lambda*sqrt(1.0-pow(lambda, 2));
    // T forx=1 (parabolic case)
    double T1 = 2.0/3.0*(1.0 - pow(lambda, 3));
    // T for x=0 and M revolutions
    double T0M = T00 + this->M*M_PI;
    
    double x0_1 = 0.0;
    double x0_2 = 0.0;
    
    if (this->solverName == "Gooding"){
        // inital guess procedure for Gooding algorithm
        double c0 = 1.7;
        double c1 = 0.5;
        double c2 = 0.03;
        double c3 = 0.15;
        double c41 = 1.0;
        double c42 = 0.24;
        
        if (this->M == 0){
            // zero revolution case
            
            double x01 = -(T - T00)/(T - T00 + 4.0);
            double theta = 2.0*acos(lambda/(1.0-this->c/(2.0*this->s)));
            // theta between 0 and pi if lambda > 0 and between pi and 2pi if lambda < 0
            if (lambda < 0){
                theta = 2.0*M_PI - theta;
            }
            double W = x01 + 1.7*sqrt(2.0-theta/M_PI);
            double x03 = 0.0;
            if (W >= 0.0) {
                x03 = x01;
            }
            else {
                double x02 = -sqrt((T - T00)/(T + 1.0/2.0*T00));
                double w = pow(-W,1.0/16.0);
                x03 = x01 + w*(x02 - x01);
            }
            double scale = 1.0 + c1*x03*(1.0 + x01) - c2*pow(x03,2)*sqrt(1.0 + x01);
            
            x0_1 = scale*x03;
        }
        else {
            // multi-revoluion case
            
            double theta = 2.0*atan2(2.0*lambda, 1.0-pow(lambda,2));
            double x_M_pi = 4.0/(3.0*M_PI*(2*this->M + 1));
            double x_M = 0.0;
            if (theta <= M_PI){
                x_M = x_M_pi*pow(theta/M_PI, 1.0/8.0);
            }
            else {
                x_M = x_M_pi*(2.0 - pow(2.0 - theta/M_PI, 1.0/8.0));
            }

            double Tmin = this->getTmin(T0M, 0.0, this->M, 1e-5, 12);
            std::cout << "Tmin: " << Tmin << std::endl;
            std::cout << "T: " << T << std::endl;
            // get derivatives of T at x_M
            std::vector<double> DTs = dTdx(x_M, Tmin, this->lambda);
            double D2T = DTs[1];
                
            if (T > Tmin){
                // if T > Tmin, two multi-revolution solutions exist for the given time of flight T (or one solution if T = Tmin)
                this->noMultiRevSolution = false;
                
                double TdiffM = T - Tmin;
                double Tdiff = T - T0M;
                
                if (T > T0M){
                    double term1 = TdiffM/(0.5*D2T - TdiffM*(0.5*D2T/(T0M-Tmin)-1/pow(x_M,2)));
                    x0_1 = x_M - pow(term1, 1.0/2.0);
                }
                else{
                    double x01_1 =  -Tdiff/(Tdiff + 4.0);
                    double W = x01_1 + c0*pow(2.0*(1.0-theta/(2*M_PI)),1.0/2.0);
                    if (W < 0.0){
                        x0_1 = x01_1 - pow(-W, 1.0/16.0)*(x01_1 + pow(Tdiff/(Tdiff+1.5*T0M),1.0/2.0));
                    }
                    else{
                        double W2 = 4.0/(4.0+Tdiff);
                        x0_1 = x01_1*(1.0+(1.0+this->M+c42*(theta/(2*M_PI)-0.5))/(1.0-c3*this->M)*x01_1*(c1*W2-c2*x01_1*pow(W2,1.0/2.0)));
                    }
                }
                
                double term2 = TdiffM/(0.5*D2T + TdiffM/pow(1-x_M,2));
                double x01_2 = x_M + pow(term2, 1.0/2.0);
                double W = x_M + x01_2;
                W = W*4.0/(4.0 + TdiffM) + pow(1.0-W,2);
                x0_2 = x01_2*(1.0-(1.0+this->M+c41*(theta/(2*M_PI)-0.5))/(1.0-c3*this->M)*x01_2*(c1*W+c2*x01_2*pow(W,1.0/2.0))) - x_M;
            }
            else{
                // if T < Tmin, no multi-revolution solution exists for the given time of flight T
                
                this->noMultiRevSolution = true;
                bskLogger.bskLog(BSK_WARNING, "lambertSolver: no multi-revolution solution exists for the given time of flight.");
            }
        }
    }
    else if (this->solverName == "Izzo"){
        // inital guess procedure for Izzo algorithm
        
        if (this->M == 0){
            // zero revolution case
            
            if (T >= T00){
                x0_1 = pow(T00/T, 2.0/3.0) - 1.0;
            }
            else if (T <= T1){
                x0_1 = 5.0/2.0*T1*(T1 - T)/(T*(1.0 - pow(lambda, 5))) + 1.0;
            }
            else{
                x0_1 = pow(T00/T, log(T1/T00)/log(2.0)) - 1.0;
            }
        }
        else{
            // multi-revolution case
            
            double Tmin = this->getTmin(T0M, 0.0, this->M, 1e-5, 12);
            std::cout << "Tmin: " << Tmin << std::endl;
            if (T >= Tmin){
                // if T > Tmin, two multi-revolution solutions exist for the given time of flight T (or one solution if T = Tmin)
                
                this->noMultiRevSolution = false;
                
                double term1 = pow((this->M*M_PI + M_PI)/(8.0*T),(2.0/3.0));
                double term2 = pow((8.0*T)/(this->M*M_PI),(2.0/3.0));
                x0_1 = (term1 - 1.0)/(term1 + 1.0);
                x0_2 = (term2 - 1.0)/(term2 + 1.0);
            }
            else{
                // if T < Tmin, no multi-revolution solution exists for the given time of flight T
                
                this->noMultiRevSolution = true;
                bskLogger.bskLog(BSK_WARNING, "lambertSolver: no multi-revolution solution exists for the given time of flight.");
            }
        }
    }
    
    std::vector<double> x0 = {x0_1, x0_2};
    return x0;
}

double LambertSolver::x2tof(double x, int N, double lambda)
{
    double battin = 0.01;
    double dist = abs(x - 1.0);
    double u = 1.0 - pow(x, 2);
    double y = sqrt(1.0-pow(lambda,2)*u);
    
    double tof = 0.0;
    if (dist < battin){
        // Use Battin formulation
        
        double eta = y - lambda * x;
        double S1 = 0.5 * (1.0 - lambda - x * eta);
        double Q = 4.0 / 3.0 * hypergeometricF(3.0, 1.0, 2.5, S1, 1e-11);
        tof = (pow(eta,3)*Q + 4.0*lambda*eta)/2.0 + N*M_PI/(pow(abs(u),1.5));
    }
    else{
        // Use Lancaster formulation
        
        double f = (y - lambda*x)*sqrt(abs(u));
        double g = x*y + lambda*u;
        double d = 0.0;
        if (u > 0.0){
            // elliptic case
            double psi = atan2(f,g);
            d = psi + N*M_PI;
        }
        else{
            // hyperbolic case
            d = atanh(f/g);
        }
        
        tof = (d/sqrt(abs(u)) - x + lambda*y)/u;
    }
    
    return tof;
}

std::vector<double> LambertSolver::dTdx(double x, double T, double lambda)
{
    double u = 1.0 - pow(x, 2);
    double y = sqrt(1.0-pow(lambda,2)*u);

    // dT/dx
    double DT = 1.0/u*(3.0*T*x - 2.0 + 2.0*pow(lambda,3)*x/y);
    // d2T/dx2
    double D2T = 1.0/u*(3.0*T + 5.0*x*DT + 2.0*(1. - pow(lambda,2))*pow(lambda,3)/pow(y,3));
    // d3T/dx3
    double D3T = 1.0/u*(7.0*x*D2T + 8.0*DT - 6.0*(1. - pow(lambda,2))*pow(lambda,5)*x/pow(y,5));
    
    std::vector<double> DTs = {DT, D2T, D3T};
    
    return DTs;
}

double LambertSolver::householder(double T, double x0, int N, double tol, int iter_max)
{
    double xnew = 0.0; // initialize
    
    for (int j = 0; j <= iter_max; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        double D3T = DTs[2];
        
        double delta = tof - T;
        // compute new x using 3rd order householder algorithm
        xnew = x0 - delta*(pow(DT,2) - delta*D2T/2.0)/(DT*(pow(DT,2) - delta*D2T) + D3T*pow(delta,2)/6.0);
        
        double err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
    }
   
    return xnew;
}

double LambertSolver::halley(double T, double x0, int N, double tol, int iter_max)
{
    double xnew = 0.0; // initialize
    
    for (int j = 0; j <= iter_max; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        
        double delta = tof - T;
        // compute new x using halley algorithm
        xnew = x0 - delta*DT/(pow(DT,2) - delta*D2T/2.0);
        double err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
    }
   
    return xnew;
}

double LambertSolver::getTmin(double T0M, double x0, int N, double tol, int iter_max)
{
    double xnew = 0.0; // initialize
    double tof = T0M;
    
    for (int j = 0; j <= iter_max; ++j){
        // get derivatives of T
        std::vector<double> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        double D3T = DTs[2];
        
        // compute new x using halley algorithm
        xnew = x0 - DT*D2T/(pow(D2T,2) - DT*D3T/2.0);
        double err = abs(x0 - xnew);
        x0 = xnew;
        if (err < tol){
            break;
        }
        // compute non-dimensional time-of-flight T for given x
        tof = x2tof(xnew, N, this->lambda);
    }
    double Tmin = tof;
   
    return Tmin;
}

double LambertSolver::hypergeometricF(double a, double b, double c, double z, double tol)
{
    double Sj = 1.0;
    double Cj = 1.0;
    double Cj1 = 0.0;
    double Sj1 = 0.0;
    double err = 1.0;
    for (int j = 0; j <= 12; ++j){
        Cj1 = Cj * (a + j) * (b + j) / (c + j) * z / (j + 1);
        Sj1 = Sj + Cj1;
        err = abs(Cj1);
        Sj = Sj1;
        Cj = Cj1;
        if (err < tol){
            break;
        }
    }
    return Sj;
}
