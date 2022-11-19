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

# update "module" in this function name to reflect the module name
def test_Prescribed1DOFTestFunction(show_plots, thetaRef, thetaDotRef, accuracy):

    # each test method requires a single assert method to be called
    [testResults, testMessage] = Prescribed1DOFTestFunction(show_plots, thetaRef, thetaDotRef, accuracy)

    assert testResults < 1, testMessage


def Prescribed1DOFTestFunction(show_plots, thetaRef, thetaDotRef, accuracy):

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
    # These will eventually become input messages
    spinAxis = 0
    Prescribed1DOFConfig.thetaDDotMax = 0.0087  # [rad/s^2]
    Prescribed1DOFConfig.spinAxis = spinAxis

    # Create input message
    thetaRef = np.pi/3
    thetaDotRef = 0.0
    thetaInit = 0.0
    thetaDotInit = 0.0

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
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(60.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Extract data for unit test
    omega_FB_F = dataLog.omega_FB_F
    thetaDotFinal_sim = omega_FB_F[-1, spinAxis]

    sigma_FB = dataLog.sigma_FB
    sigma_FBFinal_sim = sigma_FB[-1, :]

    thetaFinal_sim = 4 * np.arctan(sigma_FBFinal_sim[spinAxis])

    # Determine thetaSim
    theta_FB_sim = 4 * np.arctan(sigma_FB[:, spinAxis])

    timespan = dataLog.times()

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, omega_FB_F[:, 0],
             timespan * 1e-9, omega_FB_F[:, 1],
             timespan * 1e-9, omega_FB_F[:, 2])
    plt.xlabel('Time (s)')
    plt.ylabel('Prescribed omega_FB_F')

    # Plot simulated theta_FB
    thetaRef_plotting = np.ones(len(timespan)) * thetaRef
    thetaInit_plotting = np.ones(len(timespan)) * thetaInit

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, theta_FB_sim,
             timespan * 1e-9, thetaRef_plotting, '--',
             timespan * 1e-9, thetaInit_plotting, '--')
    plt.xlabel('Time (s)')
    plt.ylabel('theta_FB_sim')

    if show_plots:
        plt.show()
    plt.close("all")

    # set the filtered output truth states
    if not unitTestSupport.isDoubleEqual(thetaDotFinal_sim, thetaDotRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "thetaDotFinal_sim and thetaDotRef do not match")

    if not unitTestSupport.isDoubleEqual(thetaFinal_sim, thetaRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "thetaFinal_sim and thetaRef do not match")

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    Prescribed1DOFTestFunction(
                 True,
                 np.pi/3,     # thetaRef
                 0,           # thetaDotRef
                 1e-12        # accuracy
               )
