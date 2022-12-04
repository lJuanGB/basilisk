#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        prescribed1DOF
#   Author:             Leah Kiner
#   Creation Date:      Nov 14, 2022
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.utilities import RigidBodyKinematics as rbk
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import prescribed1DOF                # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging

ang = np.linspace(0, np.pi, 5)  # [rad]
ang = list(ang)
maxAngAccel = np.linspace(0.008, 0.015, 3)  # [rad/s^2]
maxAngAccel = list(maxAngAccel)

@pytest.mark.parametrize("thetaInit", ang)
@pytest.mark.parametrize("thetaRef", ang)
@pytest.mark.parametrize("thetaDDotMax", maxAngAccel)
@pytest.mark.parametrize("accuracy", [1e-12])

# update "module" in this function name to reflect the module name
def test_Prescribed1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    # each test method requires a single assert method to be called
    [testResults, testMessage] = Prescribed1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy)

    assert testResults < 1, testMessage


def Prescribed1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    testFailCount = 0                                        # zero unit test result counter
    testMessages = []                                        # create empty array to store test log messages
    unitTaskName = "unitTask"                                # arbitrary name (don't change)
    unitProcessName = "TestProcess"                          # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    Prescribed1DOFConfig = prescribed1DOF.Prescribed1DOFConfig()
    PrescribedWrap = unitTestSim.setModelDataWrap(Prescribed1DOFConfig)
    PrescribedWrap.ModelTag = "Prescribed1DOF"                                 # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedWrap, Prescribed1DOFConfig)

    # Initialize the test module configuration data
    Prescribed1DOFConfig.rotAxis_M = np.array([1.0, 0.0, 0.0])
    Prescribed1DOFConfig.thetaDDotMax = thetaDDotMax  # [rad/s^2]
    Prescribed1DOFConfig.r_FM_M = np.array([1.0, 0.0, 0.0])
    Prescribed1DOFConfig.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    Prescribed1DOFConfig.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    Prescribed1DOFConfig.omega_FM_F = np.array([0.0, 0.0, 0.0])
    Prescribed1DOFConfig.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    Prescribed1DOFConfig.sigma_FM = np.array([0.0, 0.0, 0.0])

    # Create input message
    thetaDotRef = 0.0  # [rad/s]
    thetaDotInit = 0.0  # [rad/s^2]

    SpinningBodyMessageData = messaging.SpinningBodyMsgPayload()
    SpinningBodyMessageData.theta = thetaRef
    SpinningBodyMessageData.thetaDot = thetaDotRef
    SpinningBodyMessage = messaging.SpinningBodyMsg().write(SpinningBodyMessageData)
    Prescribed1DOFConfig.spinningBodyInMsg.subscribeTo(SpinningBodyMessage)

    # Connect module output message to its own input message
    Prescribed1DOFConfig.prescribedMotionInMsg.subscribeTo(Prescribed1DOFConfig.prescribedMotionOutMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = Prescribed1DOFConfig.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    simTime = np.sqrt(((0.5 * np.abs(thetaRef - thetaInit)) * 8) / thetaDDotMax)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    SpinningBodyMessageData = messaging.SpinningBodyMsgPayload()
    SpinningBodyMessageData.theta = 2 * thetaRef
    SpinningBodyMessageData.thetaDot = thetaDotRef
    SpinningBodyMessage = messaging.SpinningBodyMsg().write(SpinningBodyMessageData, macros.sec2nano(simTime))
    Prescribed1DOFConfig.spinningBodyInMsg.subscribeTo(SpinningBodyMessage)
    unitTestSim.ConfigureStopTime(2 * macros.sec2nano(simTime))  # seconds to stop simulation

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
    Prescribed1DOFTestFunction(
                 True,
                 0.0,         # thetaInit
                 np.pi/3,     # thetaRef
                 0.008,       # thetaDDotMax
                 1e-12        # accuracy
               )
