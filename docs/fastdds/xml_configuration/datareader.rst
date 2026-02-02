.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _subscriberprofiles:

DataReader profiles
-------------------

The DataReader profiles allow declaring |DataReaders| from an XML file.
These profiles are defined within the ``<data_reader>`` XML tags.

DataReader XML attributes
^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<data_reader>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``profile_name``
     - Sets the name under which the ``<data_reader>`` profile is registered in the DDS Domain,
       so that it can be loaded later by the |DomainParticipant|, as shown in
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<data_reader>`` profile as the default profile. Thus, if a default profile
       exists, it will be used when no other DataReader profile is specified at the
       DataReader's creation.
     - Optional

DataReader configuration
^^^^^^^^^^^^^^^^^^^^^^^^

The DataReader configuration is performed through the XML elements listed in the following table.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<topic>``
     - :ref:`TopicType` configuration of the DataReader.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - Subscriber :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It allows configuring some time related
       parameters of the DataReader.
     - :ref:`ReaderTimes <subtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators.
       It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators.
       It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<external_unicast_locators>``
     - List of :ref:`external_locators`
       to announce for the communication
       with this DataReader.
     - :ref:`externalLocatorListType`
     -
   * - ``<ignore_non_matching_locators>``
     - Whether to ignore locators received on
       announcements from other entities when
       they don't match with any of the locators
       announced by this DataReader.
     - ``bool``
     - false
   * - ``<expects_inline_qos>``
     - It indicates if QoS is expected inline.
     - ``bool``
     - ``false``
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for DataReaders's  history.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for StaticEndpointDiscovery.
     - ``int16_t``
     - -1
   * - ``<entityID>``
     - Set the |RTPSEndpointQos::entity_id-api| of the |RTPSEndpointQos|  class.
     - ``int16_t``
     - -1
   * - ``<matchedPublishersAllocation>``
     - Sets the limits of the collection of matched
       DataWriters. See
       :ref:`participantresourcelimitsqos`.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-SUBSCRIBER<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-78, 80-81

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`EXTERNAL_LOCATOR_LIST` means a :ref:`externalLocatorListType` is expected.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _subtimes:

ReaderTimes
"""""""""""

These parameters are included within :ref:`rtpsreliablereaderqos` in the :ref:`readertimes` structure.

+--------------------------------+-----------------------------------------------------+---------------------+---------+
| Name                           | Description                                         | Values              | Default |
+================================+=====================================================+=====================+=========+
| ``<initial_acknack_delay>``    | Initial ACKNACK delay.                              | :ref:`DurationType` | 70 ms   |
+--------------------------------+-----------------------------------------------------+---------------------+---------+
| ``<heartbeat_response_delay>`` | Response time delay when receiving a Heartbeat.     | :ref:`DurationType` | 5 ms    |
+--------------------------------+-----------------------------------------------------+---------------------+---------+
