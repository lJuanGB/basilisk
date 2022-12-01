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
#   Module Name:        prescribedMotion and prescribed1DOF integrated unit test
#   Author:             Leah Kiner
#   Creation Date:      Nov 20, 2022
#
import pytest
import inspect
import os

import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import prescribed1DOF
from Basilisk.simulation import spacecraft, prescribedMotionStateEffector, gravityEffector
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging

ang = np.linspace(np.pi/2, np.pi, 2)  # [rad]
ang = list(ang)
maxAngAccel = np.linspace(0.008, 0.015, 2)  # [rad/s^2]
maxAngAccel = list(maxAngAccel)

@pytest.mark.parametrize("thetaInit", ang)
@pytest.mark.parametrize("thetaRef", ang)
@pytest.mark.parametrize("thetaDDotMax", maxAngAccel)
@pytest.mark.parametrize("accuracy", [1e-12])

# update "module" in this function name to reflect the module name
def test_PrescribedMotionTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    # each test method requires a single assert method to be called
    [testResults, testMessage] = PrescribedMotionTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy)

    assert testResults < 1, testMessage


def PrescribedMotionTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testIncrement = 0.5
    testProcessRate = macros.sec2nano(testIncrement)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # ** ** ** ** ** Add spacecraft module to test file ** ** ** ** **
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test modules to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # ** ** ** ** ** Add prescribedMotion module to test file ** ** ** ** **
    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()

    # Define properties of state effector
    platform.mass = 100.0
    platform.IPntFc_F = [[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    platform.r_MB_B = [1.0, 0.0, 0.0]
    platform.r_FcF_F = [0.0, 0.0, 0.0]
    platform.r_FM_M = [1.0, 0.0, 0.0]
    platform.rPrime_FM_M = [0.0, 0.0, 0.0]
    platform.rPrimePrime_FM_M = [0.0, 0.0, 0.0]
    platform.omega_FM_F = [0.0, 0.0, 0.0]
    platform.omegaPrime_FM_F = [0.0, 0.0, 0.0]
    platform.sigma_FM = [0.0, 0.0, 0.0]
    platform.omega_MB_B = [0.0, 0.0, 0.0]
    platform.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    platform.sigma_MB = [0.0, 0.0, 1.0]
    platform.ModelTag = "Platform"

    # Add platform to spacecraft
    scObject.addStateEffector(platform)

    # Add test modules to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, platform)

    # ** ** ** ** ** Add prescribed1DOF module to test file ** ** ** ** **
    Prescribed1DOFConfig = prescribed1DOF.Prescribed1DOFConfig()
    PrescribedWrap = unitTestSim.setModelDataWrap(Prescribed1DOFConfig)
    PrescribedWrap.ModelTag = "Prescribed1DOF"  # update python name of test module

    # Initialize the test module configuration data
    Prescribed1DOFConfig.thetaDDotMax = thetaDDotMax  # [rad/s^2]
    Prescribed1DOFConfig.rotAxis_M = np.array([1.0, 0.0, 0.0])

    # Add test modules to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedWrap, Prescribed1DOFConfig)

    # Create input message
    thetaDotRef = 0.0  # [rad/s]
    thetaDotInit = 0.0  # [rad/s^2]

    SpinningBodyMessageData = messaging.SpinningBodyMsgPayload()
    SpinningBodyMessageData.theta = thetaRef
    SpinningBodyMessageData.thetaDot = thetaDotRef
    SpinningBodyMessage = messaging.SpinningBodyMsg().write(SpinningBodyMessageData)
    Prescribed1DOFConfig.spinningBodyInMsg.subscribeTo(SpinningBodyMessage)

    # connect prescribedMotionStateEffector input message to the prescribed1DOF module output message
    platform.prescribedMotionInMsg.subscribeTo(Prescribed1DOFConfig.prescribedMotionOutMsg)
    Prescribed1DOFConfig.prescribedMotionInMsg.subscribeTo(platform.prescribedMotionOutMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = Prescribed1DOFConfig.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    earthGravBody.useSphericalHarmParams = False
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    scStateData = scObject.scStateOutMsg.recorder()

    # Add test modules to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scStateData)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    simTime = np.sqrt(((0.5 * np.abs(thetaRef - thetaInit)) * 8) / thetaDDotMax) + 45
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    omega_FM_F = dataLog.omega_FM_F
    sigma_FM = dataLog.sigma_FM
    timespan = dataLog.times()

    thetaDot_Final = np.linalg.norm(omega_FM_F[-1, :])
    sigma_FM_Final = sigma_FM[-1, :]

    # Convert MRPs to theta_FM
    n = len(timespan)
    theta_FM = []
    for i in range(n):
        theta_FM.append(4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))

    theta_FM_Final = 4 * np.arctan(np.linalg.norm(sigma_FM_Final))

    # Plot omega_FB_F
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, omega_FM_F[:, 0],
             timespan * 1e-9, omega_FM_F[:, 1],
             timespan * 1e-9, omega_FM_F[:, 2])
    plt.xlabel('Time (s)')
    plt.ylabel('omega_FB_F (rad/s)')

    # Plot simulated theta_FM
    thetaRef_plotting = np.ones(len(timespan)) * thetaRef
    thetaInit_plotting = np.ones(len(timespan)) * thetaInit
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, theta_FM, label='theta_FM')
    plt.plot(timespan * 1e-9, thetaRef_plotting, '--', label='thetaRef')
    plt.plot(timespan * 1e-9, thetaInit_plotting, '--', label='thetaInit')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta_FM (rad)')
    plt.legend()

    if show_plots:
        plt.show()
    plt.close("all")

    # set the filtered output truth states
    if not unitTestSupport.isDoubleEqual(thetaDot_Final, thetaDotRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "thetaDot_Final and thetaDotRef do not match")

    if not unitTestSupport.isDoubleEqual(theta_FM_Final, thetaRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "theta_FM_Final and thetaRef do not match")

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    PrescribedMotionTestFunction(
                 True,
                 0.0,         # thetaInit
                 np.pi/3,     # thetaRef
                 0.008,       # thetaDDotMax
                 1e-12        # accuracy
               )
