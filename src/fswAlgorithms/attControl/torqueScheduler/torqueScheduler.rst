Executive Summary
-----------------

This module schedules two control torques such that only one is actually active at a time. This is useful in the case of a system
with multiple coupled degrees of freedom, where the changes in one controlled variable can affect the other controlled variable and thus
cause the system to not converge. 


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - Output Array Motor Torque Message.
    * - motorTorque1InMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - #1 Input Array Motor Torque Message.
    * - motorTorque2InMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - #2 Input Array Motor Torque Message. 


Module Assumptions and Limitations
----------------------------------
This module is very simple and does not make any assumptions.


Detailed Module Description
---------------------------
This module receives a ``tSwitch`` parameter as an input from the user. This parameter determines the simulation time at which the first motor torque is disabled and the second is enabled. Both torques are stored at all times in ``motorTorqueOutMsg.motorTorque``. The logic in this module
allocates the flag 0 or 1 in ``motorTorqueOutMsg.motorLockFlag`` to determine whether that torque is to be applied (``motorLockFlag = 0``) or neglected (``motorLockFlag = 1``). For ``t < tSwitch``, the first input torque is applied; for ``t > tSwitch``, the second input torque is applied.


User Guide
----------
The required module configuration is::

    schedulerConfig = torqueScheduler.torqueSchedulerConfig()
    schedulerWrap = unitTestSim.setModelDataWrap(schedulerConfig)
    schedulerWrap.ModelTag = "torqueScheduler"
    schedulerConfig.tSwitch = tSwitch
    unitTestSim.AddModelToTask(unitTaskName, schedulerWrap, schedulerConfig)
	
The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 34 66
   :header-rows: 1

   * - Parameter
     - Description
   * - ``tSwitch``
     - time at which the torque is switched from input 1 to input 2. If not provided it defaults to zero, therefore only input torque is passed.
