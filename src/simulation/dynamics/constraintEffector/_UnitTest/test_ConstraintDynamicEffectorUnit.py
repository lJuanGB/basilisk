# ISC License
#
# Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        constraintEffector
#   Author:             João Vaz Carneiro
#   Creation Date:      November 8, 2022
#

import inspect
import os

import numpy

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, orbitalMotion, macros, RigidBodyKinematics, vizSupport
from Basilisk.simulation import spacecraft, constraintDynamicEffector, gravityEffector, svIntegrators
import matplotlib.pyplot as plt
import numpy as np



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


def constraintEffectorTest(show_plots):
    r"""
    """
    [testResults, testMessage] = test_constraintEffector(show_plots)
    assert testResults < 1, testMessage


def test_constraintEffector(show_plots):
    # __tracebackhide__ = True

    # Test info
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(True)

    # Create test thread
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create both spacecraft
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "spacecraftBody1"
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody2"

    # Set the integrator to RKF45
    integratorObject1 = svIntegrators.svIntegratorRK4(scObject1)
    scObject1.setIntegrator(integratorObject1)
    integratorObject2 = svIntegrators.svIntegratorRK4(scObject2)
    scObject2.setIntegrator(integratorObject2)

    # Create the constraint effector module
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()

    # Define properties of the constraint
    r_P1B1_B1 = [0.2974, 0.9466, 0.1246]
    r_P2B2_B2 = [-0.2974, -0.9466, -0.1246]
    l = 0.1
    r_P2P1_B1Init = np.dot([0.76835, 0.54882, 0.32929], l)

    # Set up the constraint effector
    constraintEffector.r_P1B1_B1 = r_P1B1_B1
    constraintEffector.r_P2B2_B2 = r_P2B2_B2
    constraintEffector.r_P2P1_B1Init = r_P2P1_B1Init
    constraintEffector.alpha = 1e2
    constraintEffector.beta = 1e2
    constraintEffector.K = 100
    constraintEffector.P = 10
    constraintEffector.ModelTag = "constraintEffector"

    # Add constraints to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    earthGravBody.useSphericalHarmParams = False
    scObject1.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])
    scObject2.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Find r and v from orbital elements
    oe = orbitalMotion.ClassicElements()
    oe.a = earthGravBody.radEquator + 7500e3  # meters
    oe.e = 0.01
    oe.i = 30.0 * macros.D2R
    oe.Omega = 60.0 * macros.D2R
    oe.omega = 15.0 * macros.D2R
    oe.f = 90.0 * macros.D2R
    r_B1N_N, rDot_B1N_N = orbitalMotion.elem2rv(earthGravBody.mu, oe)
    r_B2N_N = r_B1N_N + r_P1B1_B1 + r_P2P1_B1Init - r_P2B2_B2
    rDot_B2N_N = rDot_B1N_N
    
    # rotate vectors to the N frame to solve for r_B2N_NInit
    #dcm_B1NInit = RigidBodyKinematics.MRP2C(scObject1.hub.sigma_BNInit)
    #dcm_NB1Init = dcm_B1NInit.transpose()
    #r_P1B1_NInit = dcm_NB1Init*ConstraintEffector.r_P1B1_B1
    #r_P2P1_NInit = dcm_NB1Init*constraintEffector.r_P2P1_B1Init
    #dcm_B2NInit = RigidBodyKinematics.MRP2C(scObject2.hub.sigma_BNInit)
    #dcm_NB2Init = dcm_B2NInit.transpose()
    #r_P2B2_NInit = dcm_NB2Init*constraintEffector.r_P2B2_B2
    #scObject2.hub.r_CN_NInit = scObject1.hub.r_CN_NInit + r_P1B1_NInit + r_P2P1_NInit - r_P2B2_N

    # Define mass properties of the rigid hub of both spacecraft
    scObject1.hub.mHub = 50.0
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject1.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 5.0]]
    scObject2.hub.mHub = 200.0
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject2.hub.IHubPntBc_B = [[33.0, 0.0, 0.0], [0.0, 33.0, 0.0], [0.0, 0.0, 33.0]]

    # Set the initial values for the states
    scObject1.hub.r_CN_NInit = r_B1N_N
    scObject1.hub.v_CN_NInit = rDot_B1N_N
    scObject1.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject1.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    scObject2.hub.r_CN_NInit = r_B2N_N
    scObject2.hub.v_CN_NInit = rDot_B2N_N
    scObject2.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject2.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject1)
    unitTestSim.AddModelToTask(unitTaskName, scObject2)
    unitTestSim.AddModelToTask(unitTaskName, constraintEffector)

    # Log the spacecraft state message
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog1)
    unitTestSim.AddModelToTask(unitTaskName, datLog2)

    # Add energy and momentum variables to log
    unitTestSim.AddVariableForLogging(constraintEffector.ModelTag + ".psi_B1", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(constraintEffector.ModelTag + ".psiPrime_B1", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(constraintEffector.ModelTag + ".sigma_B2B1", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(constraintEffector.ModelTag + ".omega_B2B1_B2", testProcessRate, 0, 2, 'double')

    # Initialize the simulation
    # breakpoint()
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 75*60
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, [scObject1, scObject2],
                                            saveFile=__file__
                                            # liveStream=True
                                            )

    # Extract the logged variables
    psi_B1 = unitTestSim.GetLogVariableData(constraintEffector.ModelTag + ".psi_B1")
    psiPrime_B1 = unitTestSim.GetLogVariableData(constraintEffector.ModelTag + ".psiPrime_B1")
    sigma_B2B1 = unitTestSim.GetLogVariableData(constraintEffector.ModelTag + ".sigma_B2B1")
    omega_B2B1_B2 = unitTestSim.GetLogVariableData(constraintEffector.ModelTag + ".omega_B2B1_B2")

    # Grab the time vector
    timeData = psi_B1[:, 0] * macros.NANO2SEC

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    psi_B1 = np.delete(psi_B1, 0, axis=1)
    for i in range(3):
        plt.plot(timeData/60, psi_B1[:, i])
    plt.xlabel('time (minutes)')
    plt.ylabel('variation from fixed length: '+r'$\psi$'+' (meters)')
    plt.title('Length Constraint Components')

    plt.figure()
    plt.clf()
    psiPrime_B1 = np.delete(psiPrime_B1, 0, axis=1)
    for i in range(3):
        plt.plot(timeData/60, psiPrime_B1[:, i])
    plt.xlabel('time (minutes)')
    plt.ylabel('magnitude: d'+r'$\psi$'+'/dt (meters/second)')
    plt.title('Length Rate of Change Constraint Components')

    plt.figure()
    plt.clf()
    sigma_B2B1 = np.delete(sigma_B2B1, 0, axis=1)
    for i in range(3):
        plt.plot(timeData/60, 4*np.arctan(sigma_B2B1[:, i]) * macros.R2D)
    plt.xlabel('time (minutes)')
    plt.ylabel('relative attitude angle: '+r'$\phi$'+' (deg)')
    plt.title('Attitude Constraint Components')

    plt.figure()
    plt.clf()
    omega_B2B1_B2 = np.delete(omega_B2B1_B2, 0, axis=1)
    for i in range(3):
        plt.plot(timeData/60, omega_B2B1_B2[:, i] * macros.R2D)
    plt.xlabel('time (minutes)')
    plt.ylabel('angular rate: '+r'$\omega$'+' (deg/second)')
    plt.title('Attitude Rate Constraint Components')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    if testFailCount == 0:
        print("PASSED: " + " Constraint Effector integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    constraintEffectorTest(True)
