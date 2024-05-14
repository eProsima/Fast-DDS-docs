.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _property_policies_non_consolidated_qos:

Non consolidated QoS
--------------------

The :ref:`property_policies` are used to develop new :ref:`eprosima_extensions` QoS.
Before consolidating a new QoS Policy, it is usually set using this generic QoS Policy.
Consequently, this section is prone to frequent updates so the user is advised to check latest changes after upgrading
to a different release version.

.. _push_mode_qos_policy:

``DataWriter operating mode`` QoS Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, *Fast DDS* |DataWriters| are enabled using ``push mode``.
This implies that they will add new samples into their queue, and then immediately deliver them to matched readers.
For writers that produce non periodic bursts of data, this may imply saturating the network with a lot of packets,
increasing the possibility of losing them on unreliable (i.e. UDP) transports.
Depending on their QoS, DataReaders may also have to ignore some received samples, so they will have to be resent.

Configuring the DataWriters on ``pull mode`` offers an alternative by letting each reader pace its own data stream.
It works by the writer notifying the reader what it is available, and waiting for it to request only as much as it
can handle.
At the cost of greater latency, this model can deliver reliability while using far fewer packets than ``push mode``.

DataWriters periodically announce the state of their queue by means of a heartbeat.
Upon reception of the heartbeat, DataReaders will request the DataWriter to send the samples they want to process.
Consequently, the publishing rate can be tuned setting the heartbeat period accordingly.
See :ref:`tuning-heartbeat-period` for more details.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.push_mode"``
     - ``"true"``/``"false"``
     - ``"true"``

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // PULL_MODE_DATAWRITER                                                                             |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->PULL_MODE_DATAWRITER<-->                                                                       |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-                                                                                                    |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

.. note::
    * Communication to readers running on the same process (:ref:`intraprocess-delivery`) will always use ``push mode``.
    * Communication to |BEST_EFFORT_RELIABILITY_QOS-api| readers will always use ``push mode``.

.. warning::
    * It is inconsistent to enable the ``pull mode`` and also set the |ReliabilityQosPolicyKind-api| to
      |BEST_EFFORT_RELIABILITY_QOS-api|.
    * It is inconsistent to enable the ``pull mode`` and also set the |WriterTimes::heartbeatPeriod-api| to
      |c_TimeInfinite-api|.


Unique network flows QoS Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This section is still under work.

.. _property_policies_statistics:

Statistics Module Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS Statistics Module uses the :ref:`propertypolicyqos` to indicate the statistics DataWriters that are enabled
automatically (see :ref:`auto_enabling_statistics_datawriters`).
In this case, the property value is a semicolon separated list containing the
:ref:`statistics topic name aliases<statistics_topic_names>` of those DataWriters that the user wants to enable.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.statistics"``
     - Semicolon separated list of :ref:`statistics topic name aliases<statistics_topic_names>`
     - ``""``

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // FASTDDS_STATISTICS_MODULE                                                                        |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->FASTDDS_STATISTICS_MODULE<-->                                                                  |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _property_policies_physical_data:

Physical Data in Discovery Information
""""""""""""""""""""""""""""""""""""""

It is possible to include the information conveyed in the :ref:`statistics_topic_names_physical` into the participant
discovery message, a.k.a *DATA[p]* (see :ref:`disc_phases`).
This is done by setting the following properties within the :ref:`propertypolicyqos`:

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value without |br| ``FASTDDS_STATISTICS``
     - Default value with |br| ``FASTDDS_STATISTICS``
   * - ``"fastdds.physical_data.host"``
     - Name of the host computer in which |br| the application runs
     - Not set
     - ``""``
   * - ``"fastdds.physical_data.user"``
     - Name of the user running the application
     - Not set
     - ``""``
   * - ``"fastdds.physical_data.process"``
     - Name of the process running the application
     - Not set
     - ``""``

Whenever any of these properties is defined within the |DomainParticipantQos-api|, the |DomainParticipant-api| *DATA[p]*
will contained the set value.
Furthermore, if any of these properties is set to a value of ``""``, which is the default when ``FASTDDS_STATISTICS`` is
defined (see :ref:`cmake_options`), *Fast DDS* will automatically populate the value using the following convention:

* ``"fastdds.physical_data.host"``: Host name as returned by `asio::ip::host_name() <https://think-async.com/Asio/asio-1.22.0/doc/asio/reference/ip__host_name.html>`_,
  followed by ``":<default data sharing domain id>"``
* ``"fastdds.physical_data.user"``: Name of the user running the application, or ``"unknown"`` if it could not be
  retrieved.
* ``"fastdds.physical_data.process"``: The process ID of the process in which the application is running.

All the previous entails that adding physical information to the *DATA[p]* can be done regardless of whether
``FASTDDS_STATISTICS`` is defined, and that it is possible to let *Fast DDS* set some default values into the
reported ``host``, ``user``, and ``process``:

1. If ``FASTDDS_STATISTICS`` is defined, and the user does not specify otherwise, *Fast DDS* will set default values to
   the physical properties of the *DATA[p]*.

2. If ``FASTDDS_STATISTICS`` is defined, and the user sets values to the properties, the user settings are honored.

3. If ``FASTDDS_STATISTICS`` is defined, and the user removes the physical properties from the
   |DomainParticipantQos-api|, then no physical information is transmitted in the *DATA[p]*.

4. If ``FASTDDS_STATISTICS`` is not defined, it is still possible to transmit physical information in the *DATA[p]* by
   setting the aforementioned properties:

   a) If set to ``""``, then *Fast DDS* will populate their value according to the described rules.
   b) If set to something other than ``""``, then the set value will be transmitted in the *DATA[p]* as-is.

In case ``FASTDDS_STATISTICS`` is defined, and the reporting of statistics over the ``DISCOVERY_TOPIC`` is enabled (see
:ref:`property_policies_statistics`), then the physical information included in the *DATA[p]* is also transmitted over
the ``DISCOVERY_TOPIC`` (see :ref:`statistics_topic_names_physical`) whenever one |DomainParticipant-api| discovers
another one.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: // FASTDDS_PHYSICAL_PROPERTIES
       :end-before: //!--
       :dedent: 8

  .. tab:: XML

   .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->FASTDDS_PHYSICAL_PROPERTIES<-->
      :end-before: <!--><-->
      :lines: 2-4,6-25,27-28

.. important::

  The properties set using XML override those in the default QoS, which means that it is possible to set the physical
  properties using XML regardless of whether ``FASTDDS_STATISTICS`` is defined.
  However, it is not possible to remove the properties using XML, meaning that an application using *Fast DDS* with
  ``FASTDDS_STATISTICS`` enabled which does not want for the physical information to be transmitted in the
  |DomainParticipant-api| *DATA[p]* must remove the properties using the aforementioned `C++` API.

.. _property_policies_partitions:

Endpoint Partitions
^^^^^^^^^^^^^^^^^^^

Fast DDS uses this :ref:`propertypolicyqos` to define which partitions does an endpoint belong to. This property
follows the same logic regarding matching as the |PartitionQosPolicy| that can be defined for Publishers and
Subscribers.

This property's value is a semicolon separated list containing the partition names the user wants this endpoint
to belong to.

.. important::
     If both a Publisher and one of its DataWriters have conflicting partition configuration, this is, a DataWriter
     has this property defined while the Publisher has the |PartitionQosPolicy| defined, the DataWriter
     configuration takes precedence and the Publisher |PartitionQosPolicy| is ignored for this endpoint. This applies
     to Subscribers and their DataReaders as well.

     This property will be automatically set when creating DataReaders and DataWriters using the create_with_profile
     functions. It cannot be changed after the entity has been created.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"partitions"``
     - Semicolon separated list of partition names
     - ``""``

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // PARTITION-ON-ENDPOINT                                                                            |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->XML-PARTITION                                                                                  |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _property_policies_edp_exchange_format:

Static Discovery's Exchange Format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Static Discovery exchanges data in the Participant Discovery Phase (PDP).
Currently there are two different exchange formats which can be selected using the property
``dds.discovery.static_edp.exchange_format``.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos value
     - Description
     - Default
   * - ``"v1"``
     - Standard exchange format for Static Discovery.
     - ✅
   * - ``"v1_Reduced"``
     - Format which reduces the necessary network bandwidth to transmit Static |br|
       Discovery's information in the Participant Discovery Phase (PDP).
     -

.. tabs::

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: //DDS-STATIC-DISCOVERY-FORMAT
           :end-before: //!--
           :dedent: 8

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
           :language: xml
           :start-after: <!-->XML-STATIC-DISCOVERY-FORMAT
           :end-before: <!--><-->

.. _property_policies_shm_enforce_metatraffic:

SHM transport meta-traffic enforcement
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A DomainParticipant will by default configure both a :ref:`transport_udp_udp` and a
:ref:`transport_sharedMemory_sharedMemory`.
When a participant on another process in the same host is discovered, the endpoint discovery
might be done using either transport.

Avoiding Shared Memory communication for discovery traffic can save valuable resources.
The behavior regarding this can be configured using the property ``fastdds.shm.enforce_metatraffic``.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos value
     - Description
     - Default
   * - ``"none"``
     - Use other transports for meta-traffic.
     - ✅
   * - ``"unicast"``
     - Enable SHM transport unicast communications.
     -
   * - ``"all"``
     - Enable SHM transport unicast and multicast communications. |br|
       This will enable discovery between SHM only participants |br|
       and participants having several transports.
     -

.. note::

  When SHM is the only transport configured for a participant, the setting of this property is ignored,
  and considered to be ``"all"``.

.. tabs::

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: //DDS-SHM-ENFORCE-META-TRAFFIC
           :end-before: //!--
           :dedent: 8

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
           :language: xml
           :start-after: <!-->XML-SHM-ENFORCE-META-TRAFFIC
           :end-before: <!--><-->

.. _property_max_message_size:

Maximum Message Size
^^^^^^^^^^^^^^^^^^^^

It is possible to set the maximum number of bytes of an RTPS datagram sent by an RTPSParticipant.
The value can be set in the RTPSParticipant properties or in the RTPSWriter properties.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.max_message_size"``
     - ``int32_t``
     - ``"4294967295"``

.. tabs::

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: // MAX_MESSAGE_SIZE_PROPERTY_PARTICIPANT
            :end-before: //!--
            :dedent: 6

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->MAX_MESSAGE_SIZE_PROPERTY_PARTICIPANT<-->
            :end-before: <!--><-->
            :lines: 2,4-16

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.max_message_size"``
     - ``int32_t``
     - ``"4294967295"``

.. tabs::

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: // MAX_MESSAGE_SIZE_PROPERTY_WRITER
            :end-before: //!--
            :dedent: 6

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->MAX_MESSAGE_SIZE_PROPERTY_WRITER<-->
            :end-before: <!--><-->
            :lines: 2,4-14

.. note::
    An invalid value of ``fastdds.max_message_size`` results in an exception.
