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
    testProcessRate = macros.sec2nano(1.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    Prescribed1DOFConfig = prescribed1DOF.Prescribed1DOFConfig()
    PrescribedWrap = unitTestSim.setModelDataWrap(Prescribed1DOFConfig)
    PrescribedWrap.ModelTag = "Prescribed1DOF"                                 # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedWrap, Prescribed1DOFConfig)

    # Initialize the test module configuration data
    spinAxis = 0  # (0, 1, 2) principal body axis for pure spin
    Prescribed1DOFConfig.thetaDDotMax = thetaDDotMax  # [rad/s^2]
    Prescribed1DOFConfig.spinAxis = spinAxis

    # Create input message
    thetaDotRef = 0.0  # [rad/s]
    thetaDotInit = 0.0  # [rad/s]

    RefAngleMessageData = messaging.RefAngleMsgPayload()
    RefAngleMessageData.thetaRef = thetaRef
    RefAngleMessageData.thetaDotRef = thetaDotRef
    CurrAngleMessageData = messaging.CurrAngleMsgPayload()
    CurrAngleMessageData.thetaInit = thetaInit
    CurrAngleMessageData.thetaDotInit = thetaDotInit

    RefAngleMessage = messaging.RefAngleMsg().write(RefAngleMessageData)
    CurrAngleMessage = messaging.CurrAngleMsg().write(CurrAngleMessageData)

    Prescribed1DOFConfig.refAngleInMsg.subscribeTo(RefAngleMessage)
    Prescribed1DOFConfig.currAngleInMsg.subscribeTo(CurrAngleMessage)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = Prescribed1DOFConfig.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(600.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    omega_FB_F = dataLog.omega_FB_F
    sigma_FB = dataLog.sigma_FB
    timespan = dataLog.times()

    thetaDot_Final = omega_FB_F[-1, spinAxis]
    sigma_FB_Final = sigma_FB[-1, :]

    # Convert MRPs to theta_FB
    theta_FB = 4 * np.arctan(sigma_FB[:, spinAxis])
    theta_FB_Final = 4 * np.arctan(sigma_FB_Final[spinAxis])

    # Plot omega_FB_F
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, omega_FB_F[:, 0],
             timespan * 1e-9, omega_FB_F[:, 1],
             timespan * 1e-9, omega_FB_F[:, 2])
    plt.xlabel('Time (s)')
    plt.ylabel('omega_FB_F (rad/s)')

    # Plot simulated theta_FB
    thetaRef_plotting = np.ones(len(timespan)) * thetaRef
    thetaInit_plotting = np.ones(len(timespan)) * thetaInit
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, theta_FB, label='theta_FB')
    plt.plot(timespan * 1e-9, thetaRef_plotting, '--', label='thetaRef')
    plt.plot(timespan * 1e-9, thetaInit_plotting, '--', label='thetaInit')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta_FB (rad)')
    plt.legend()

    if show_plots:
        plt.show()
    plt.close("all")

    # set the filtered output truth states
    if not unitTestSupport.isDoubleEqual(thetaDot_Final, thetaDotRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "thetaDot_Final and thetaDotRef do not match")

    if not unitTestSupport.isDoubleEqual(theta_FB_Final, thetaRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "theta_FB_Final and thetaRef do not match")

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
