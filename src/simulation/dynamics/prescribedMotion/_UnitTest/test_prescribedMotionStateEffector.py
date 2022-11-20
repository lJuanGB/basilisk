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
#   Module Name:        prescribedMotion
#   Author:             Leah Kiner
#   Creation Date:      October 11, 2022
#

import inspect
import os

import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraft, prescribedMotionStateEffector, gravityEffector
from Basilisk.utilities import macros


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


def prescribedMotionTest(show_plots):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a single-axis rotating rigid body attached to a rigid hub. The scenario
    includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``

    must be checked against their initial values.
    """
    [testResults, testMessage] = test_prescribedMotion(show_plots)
    assert testResults < 1, testMessage


def test_prescribedMotion(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testIncrement = 0.001
    testProcessRate = macros.sec2nano(testIncrement)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create prescribed state effector
    simTime1 = 5
    simTime2 = simTime1 + testIncrement

    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()

    # Choose principal body axis for prescribed rotation
    rotAxisNum = 0  # rotation about first body axis
    # rotAxisNum = 1 # rotation about second body axis
    # rotAxisNum = 2 # rotation about third body axis

    # Define properties of state effector
    platform.mass = 100.0
    platform.IHubBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    platform.IPntFc_F = [[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    platform.r_MB_B = [[1.0], [0.0], [0.0]]
    platform.r_FcF_F = [[0.0], [0.0], [0.0]]
    platform.r_FM_MInit = [[1.0], [0.0], [0.0]]
    platform.rPrime_FM_MInit = [[0.0], [0.0], [0.0]]
    platform.rPrimePrime_FM_MInit = [[0.0], [0.0], [0.0]]
    platform.theta_FBInit = 0.0
    platform.thetaDot_FBInit = 0.0
    platform.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    platform.dcm_F0B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    platform.rotAxisNum = rotAxisNum
    platform.ModelTag = "Platform"

    # Add platform to spacecraft
    scObject.addStateEffector(platform)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, platform)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    earthGravBody.useSphericalHarmParams = False
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    scStateData = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scStateData)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')

    # Add states to log
    prescribedStateData = platform.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedStateData)

    # Setup and run the simulation
    platform.u = 0.5  # [N-m]
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))
    unitTestSim.ExecuteSimulation()

    platform.u = 0.5  # [N-m]
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    omega_BN_B = scStateData.omega_BN_B

    # Setup the conservation quantities
    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
    finalOrbAngMom = [orbAngMom_N[-1]]
    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
    finalRotAngMom = [rotAngMom_N[-1]]
    initialOrbEnergy = [[orbEnergy[0, 1]]]
    finalOrbEnergy = [orbEnergy[-1]]

    # Pull time span vector
    timespan = prescribedStateData.times()

    # Determine number of elements in time span vector
    numElements = len(timespan)

    # Calculate analytic results
    uPrescribed = []  # Empty list to store prescribed control input
    theta_BN_Analytic = []  # Empty list to store analytic theta_BN
    thetaDot_BN_Analytic = []  # Empty list to store analytic thetaDot_BN
    thetaDDot_BN_Analytic = []  # Empty list to store analytic thetaDDot_BN
    theta_BN_AnalyticInit = 0.0  # Initial theta_BN
    thetaDot_BN_AnalyticInit = 0.0  # Initial thetaDot_BN

    term1 = scObject.hub.IHubPntBc_B[rotAxisNum][rotAxisNum]
    term2 = platform.IPntFc_F[rotAxisNum][rotAxisNum]

    for i in range(0, numElements):
        tt = 1e-9*timespan[i]
        if tt < simTime1:
            u = 0.5
        else:
            u = 0.5
        uPrescribed.append(u)
        term3 = - (1 / (1 - (1 / (term1 + term2)) * term2)) * (1 / (term1 + term2)) * u
        thetaDDot_BN_Analytic.append(term3)
        thetaDot_BN_Analytic.append((term3 * tt) + thetaDot_BN_AnalyticInit)
        theta_BN_Analytic.append((0.5 * term3 * tt * tt) + (thetaDot_BN_AnalyticInit * tt) + theta_BN_AnalyticInit)

    plt.close("all")

    # Plotting: analytic results
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, thetaDot_BN_Analytic)
    plt.xlabel('Time (s)')
    plt.ylabel('Analytic thetaDot_BN_B')

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, theta_BN_Analytic)
    plt.xlabel('Time (s)')
    plt.ylabel('Analytic theta_BN_B')

    # Plotting: prescribed torque
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, uPrescribed)
    plt.xlabel('Time (s)')
    plt.ylabel('Prescribed Torque')

    # Plotting: difference between numerical and analytical omega_BN_B
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, (omega_BN_B[:, rotAxisNum] - thetaDot_BN_Analytic))
    plt.xlabel('Time (s)')
    plt.ylabel('Difference Between Analytic and Simulated omega_BN_B')

    # Plotting: conservation quantities
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
             timespan * 1e-9, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
             timespan * 1e-9, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    plt.xlabel('Time (s)')
    plt.ylabel('Orbital Angular Momentum Relative Difference')

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Orbital Energy Relative Difference')

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
             timespan * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]),
             timespan * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]))
    plt.xlabel('Time (s)')
    plt.ylabel('Rotational Angular Momentum Relative Difference')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12
    finalOrbAngMom = np.delete(finalOrbAngMom, 0, axis=1)  # remove time column
    finalRotAngMom = np.delete(finalRotAngMom, 0, axis=1)  # remove time column
    finalOrbEnergy = np.delete(finalOrbEnergy, 0, axis=1)  # remove time column

    for i in range(0, len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Prescribed Motion integrated test failed orbital angular momentum unit test")

    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Prescribed Motion integrated test failed rotational angular momentum unit test")

    for i in range(0, len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Prescribed Motion integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " Prescribed Motion gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    prescribedMotionTest(True)
