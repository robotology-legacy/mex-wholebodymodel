Volume Bodies
-------------

The volume body classes enables to create and specify simple *geometric volume
body objects* for the environment scenario of the robot simulation, such as a *box*,
a *ball*, or a *cylinder*. These basic geometric forms are mostly sufficient in a
research project to create a simple scenario for a given robot model. Additionally
the defined volume bodies can be linked with certain links of the robot, such as
the hands. This enables in Matlab to simulate the manipulation of objects and their
effects in a given environment.

.. autoclass:: vbObject
   :show-inheritance:

   .. automethod:: vbObject.getDefaultScalarElement

.. autoclass:: vbSphere
   :show-inheritance:
   :inherited-members:

   .. automethod:: vbSphere.vbSphere
   .. automethod:: vbSphere.setInitFrame
   .. automethod:: vbSphere.getGObj
   .. automethod:: vbSphere.updGObj
   .. automethod:: vbSphere.drawMGrid
   .. automethod:: vbSphere.ptInObj

   .. seealso:: :class:`~WBM.vbObject`, :class:`~WBM.vbCuboid` and :class:`~WBM.vbCylinder`.

.. autoclass:: vbCylinder
   :show-inheritance:
   :inherited-members:

   .. automethod:: vbCylinder.vbCylinder
   .. automethod:: vbCylinder.setInitFrame
   .. automethod:: vbCylinder.getGObj
   .. automethod:: vbCylinder.updGObj
   .. automethod:: vbCylinder.drawMGrid
   .. automethod:: vbCylinder.ptInObj

   .. seealso:: :class:`~WBM.vbObject`, :class:`~WBM.vbCuboid` and :class:`~WBM.vbSphere`.

.. autoclass:: vbCuboid
   :show-inheritance:
   :inherited-members:

   .. automethod:: vbCuboid.vbCuboid
   .. automethod:: vbCuboid.setInitFrame
   .. automethod:: vbCuboid.getGObj
   .. automethod:: vbCuboid.updGObj
   .. automethod:: vbCuboid.drawMGrid
   .. automethod:: vbCuboid.ptInObj

   .. seealso:: :class:`~WBM.vbObject`, :class:`~WBM.vbCylinder` and :class:`~WBM.vbSphere`.
