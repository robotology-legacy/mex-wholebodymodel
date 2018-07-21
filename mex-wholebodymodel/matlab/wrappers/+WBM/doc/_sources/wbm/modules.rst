The Whole-Body Model Classes
============================

.. toctree::
   :maxdepth: 3
   :name: wbmtoc

.. automodule:: +WBM

   :mod:`+WBM` is the *namespace* (and main folder) for all classes and functions
   of the WBM-Library to avoid conflicts with other classes and functions in Matlab.

   The namespace folder contains all classes for the *model*, *configuration* and
   *model state* of the robot. In addition, the folder provides also all necessary
   classes to create *graphics objects*, *volume bodies*, *target points* and *link
   trajectories* in a simulation scenario of a given robot model.

State, Model and Configuration Classes
--------------------------------------

.. autoclass:: wbmStateParams

.. autoclass:: wbmSimEnvironment
   :show-inheritance:

.. autoclass:: wbmSimConfig
   :show-inheritance:

   .. automethod:: wbmSimConfig.setPayloadStack
   .. automethod:: wbmSimConfig.setPayloadUtilTime

   .. seealso:: :class:`~WBM.genericSimConfig`, :class:`~WBM.wbmSimBody`, :class:`~WBM.wbmSimEnvironment`,
   				 :class:`~WBM.wbmLinkTrajectory` and :class:`~WBM.wbmTargetPoint`.

.. autoclass:: wbmSimBody
   :show-inheritance:

   .. automethod:: wbmSimBody.wbmSimBody

.. autoclass:: wbmRobotParams
   :show-inheritance:

   .. automethod:: wbmRobotParams.copy

   .. seealso:: :class:`~WBM.WBM`, :class:`Interfaces.IWBM` and :class:`Interfaces.IMultChainTree`.

.. autoclass:: wbmRobotModel
   :show-inheritance:

.. autoclass:: wbmRobotDrawProp
   :show-inheritance:

.. autoclass:: wbmRobotConfig
   :show-inheritance:

.. autoclass:: wbmHumanoidConfig
   :show-inheritance:

.. autoclass:: wbmFltgBaseState
   :show-inheritance:

   .. seealso:: :class:`Interfaces.iCubWBM` and :meth:`WBMBase.getFloatingBaseState`.

.. autoclass:: wbmBody
   :show-inheritance:

   .. automethod:: wbmBody.wbmBody
   .. automethod:: wbmBody.getChainIndices
   .. automethod:: wbmBody.getJointIndex
   .. automethod:: wbmBody.getJointNames
   .. automethod:: wbmBody.getChainTable
   .. automethod:: wbmBody.getJointTable

   .. _iCub Model Naming Conventions: http://wiki.icub.org/wiki/ICub_Model_naming_conventions
   .. _yarpWholeBodyInterface.ini: https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini

.. autoclass:: genericSimConfig
   :show-inheritance:
   :inherited-members:

   .. automethod:: genericSimConfig.genericSimConfig
   .. automethod:: genericSimConfig.zoomAxes
   .. automethod:: genericSimConfig.shiftAxes
   .. automethod:: genericSimConfig.addView
   .. automethod:: genericSimConfig.createVideo
   .. automethod:: genericSimConfig.renderMode

   .. seealso:: :class:`~WBM.wbmSimConfig`, :class:`~WBM.wbmSimBody`, :class:`~WBM.wbmSimEnvironment`,
   				 :class:`~WBM.wbmLinkTrajectory` and :class:`~WBM.wbmTargetPoint`.

Default Error Messages
----------------------

.. autoclass:: wbmErrorMsg

Tool and Payload Links
----------------------

The tool and payload link classes are special classes to assign a tool or a
payload object with a specific link of the robot.

.. autoclass:: wbmToolLink
.. autoclass:: wbmPayloadLink

Graphics and Colors
-------------------

.. autoclass:: wbmGObj

   .. py:method:: getGObj(obj)

      Creates a graphics object and returns a handle of it (*abstract*).

   .. py:method:: updGObj(obj, hgo)

      Updates the data parameters of the graphics object (*abstract*).

   .. seealso:: :class:`~WBM.wbmTargetPoint`, :class:`~WBM.wbmLinkTrajectory` and :class:`~WBM.vbObject`.

.. autoclass:: wbmColor

Target Points and Link Trajectories
-----------------------------------

Special classes to create *trajectory curves* and *target points* to be reached
in the simulation by specific links of the robot.

.. autoclass:: wbmTargetPoint
   :show-inheritance:

   .. automethod:: wbmTargetPoint.wbmTargetPoint
   .. automethod:: wbmTargetPoint.getGObj
   .. automethod:: wbmTargetPoint.updGObj

   **Example:**

   .. code-block:: matlab
      :emphasize-lines: 12

      % define the target points:
      trg_pts = repmat(WBM.wbmTargetPoint, nTrg, 1);
      for i = 1:nTrg
         trg_pts(i,1).pos       = trg_pos(1:3,i);
         trg_pts(i,1).marker    = '+';
         trg_pts(i,1).mkr_color = 'red';
      end

      % setup the window and draw parameters for the WBM-simulator:
      sim_config = initSimConfigICub(robot_model.urdf_robot_name);
      sim_config = wbm_icub.setupSimulation(sim_config);
      sim_config.target_pts = trg_pts;

   .. seealso:: :class:`~WBM.wbmSimConfig` and :class:`~WBM.genericSimConfig`.

.. autoclass:: wbmLinkTrajectory
   :show-inheritance:

   .. automethod:: wbmLinkTrajectory.wbmLinkTrajectory
   .. automethod:: wbmLinkTrajectory.getGObj
   .. automethod:: wbmLinkTrajectory.updGObj

   .. seealso:: :class:`~WBM.wbmGObj`, :class:`~WBM.wbmTargetPoint`, :class:`~WBM.wbmSimConfig`
   				 and :class:`~WBM.genericSimConfig`.

.. volume body classes:
.. include:: volume_body_classes.rst

.. basic whole-body model class:
.. automodule:: +WBM.@WBMBase
.. include:: wbm_base_class.rst

.. extended whole-body model class:
.. automodule:: +WBM.@WBM
.. include:: wbm_class.rst

.. rubric:: Footnotes
.. [#f3] Details about the joint, link and frame names in particular for the
         *iCub robot model* are available at the *iCub Model Naming Conventions*:
         `<http://wiki.icub.org/wiki/ICub_Model_naming_conventions>`_.

.. [#f4] If the value is *zero*, then the robot model is undefined.

.. [#f5] The URDF-file of the given robot model must exist in the directory
         of the *yarpWholeBodyInterface* or *CoDyCo-Superbuild*.

.. [#f6] The Unified Robot Description Format (URDF) is an XML specification
         to describe a robot model.

.. [#f7] The state parameter object is defined as *empty*, if all variables of
         the given data type are empty.

.. [#f8] The calculation approach to determine if a point is inside of a cylinder
         is taken and adapted from:

            - `<https://stackoverflow.com/questions/19899612/cylinder-with-filled-top-and-bottom-in-matlab>`_ and
            - `<https://de.mathworks.com/matlabcentral/answers/43534-how-to-speed-up-the-process-of-determining-if-a-point-is-inside-a-cylinder>`_.

.. [#f9] The first 3 elements of the vector-quaternion transformation represent
		   the *position* and the last 4 elements of the vector define the
		   *orientation* in *quaternions*.

.. [#f10] The state-space form reduces (through variable substitution) the
          inhomogeneous second-order ODE to a first-order ODE
          (:cite:`Featherstone2008`, chapter 3, pp. 40-42, eq. (3.8)).

.. [#f11] Source from the *velocity control thread*:
			 `<http://wiki.icub.org/brain/velControlThread_8cpp.html>`_

