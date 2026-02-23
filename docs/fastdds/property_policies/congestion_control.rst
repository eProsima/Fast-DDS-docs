.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_congestion_control:

Congestion Control Settings |Pro|
---------------------------------

When using :ref:`Congestion Control <congestion-control>` |Pro|, the DomainParticipant may need specific parameters
to be set.
Properties related with this feature lie on the ``fastdds.congestion`` namespace.
At least one of these properties should be set in order for the congestion control to be enabled.

* Property ``fastdds.congestion.plugin`` is used to set the congestion control plugin to be used.
  If the property is not present, it will be set to ``basic``.
  See :ref:`congestion control plugins <congestion-control-plugins>` for more information about the available plugins.

* Property ``fastdds.congestion.period`` is used to set the interval in milliseconds for checking whether the bandwidth
  to certain destinations should be updated.
  If the property is not present, it will be set to ``10000`` (10 seconds).
  The minimum valid value for this property is ``1500`` (0.15 seconds)

* Property ``fastdds.congestion.initial_bandwidth`` is used to set the bandwidth that will be initially assigned to
  new destinations, in bytes per second. The user must provide a value greater than the bandwidth required to send one
  message per second.
  If the property is not present, it will be set to half the size of the socket send buffer or the size of a message,
  whichever is greater.

* Property ``fastdds.congestion.increase_multiplier`` is used to set the multiplier that will be applied to increase
  the bandwidth to a destination when no congestion is detected. Thus, it should be a value greater than 1.
  If the property is not present, it will be set to ``1.2``.

* Property ``fastdds.congestion.decrease_multiplier`` is used to set the multiplier that will be applied to decrease
  the bandwidth to a destination when congestion is detected. Thus, it should be a value between 0 and 1.
  If the property is not present, it will be set to ``0.75``.

The following is an example of how to set the congestion control up using the properties of DomainParticipantQoS.


.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: // DDS_CONGESTION_CONTROL_PROPS
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->DDS_CONGESTION_CONTROL_PROPS<-->
        :end-before: <!--><-->
