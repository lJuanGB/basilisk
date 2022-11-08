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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraft, constraintDynamicEffector, gravityEffector
from Basilisk.utilities import macros


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

    # Create the constraint effector module
    breakpoint()
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()

    # Define properties of the constraint
    constraintEffector.r_P1B1_B1 = [1, 0, 1]
    constraintEffector.r_P2B2_B2 = [0, -1, 0.5]
    constraintEffector.r_P2P1_B1Init = [0, 1, 0]
    constraintEffector.l = 1.0
    constraintEffector.ModelTag = "constraintEffector"

    # Add constraints to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

    # Define mass properties of the rigid hub of both spacecraft
    scObject1.hub.mHub = 750.0
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject2.hub.mHub = 250.0
    scObject2.hub.r_BcB_B = [[0.0], [1.0], [0.0]]
    scObject2.hub.IHubPntBc_B = [[300.0, 0.0, 0.0], [0.0, 250.0, 0.0], [0.0, 0.0, 200.0]]

    # Set the initial values for the states
    scObject1.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject1.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject1.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject1.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    scObject2.hub.r_CN_NInit = scObject1.hub.r_CN_NInit
    scObject2.hub.v_CN_NInit = scObject1.hub.v_CN_NInit
    scObject2.hub.sigma_BNInit = scObject1.hub.sigma_BNInit
    scObject2.hub.omega_BN_BInit = scObject1.hub.omega_BN_BInit

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject1)
    unitTestSim.AddModelToTask(unitTaskName, scObject2)
    unitTestSim.AddModelToTask(unitTaskName, constraintEffector)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    earthGravBody.useSphericalHarmParams = False
    # scObject1.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])
    # scObject2.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog1)
    unitTestSim.AddModelToTask(unitTaskName, datLog2)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    # Plotting
    plt.close("all")

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    # for i in range(0, len(initialOrbAngMom_N)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
    #         testFailCount += 1
    #         testMessages.append(
    #             "FAILED: Spinning Body integrated test failed orbital angular momentum unit test")
    #
    # for i in range(0, len(initialRotAngMom_N)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
    #         testFailCount += 1
    #         testMessages.append(
    #             "FAILED: Spinning Body integrated test failed rotational angular momentum unit test")
    #
    # for i in range(0, len(initialRotEnergy)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i], initialRotEnergy[i], 1, accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spinning Body integrated test failed rotational energy unit test")
    #
    # for i in range(0, len(initialOrbEnergy)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spinning Body integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spinning Body gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    constraintEffectorTest(False)