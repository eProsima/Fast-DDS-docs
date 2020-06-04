.. include:: includes/aliases.rst

.. _subscriberprofiles:

DataReader profiles
-------------------

The DataReader profiles allow declaring |DataReaders| from an XML file.
These profiles are defined within the ``<data_reader>`` or ``<subscriber>`` XML tags.
Thus, the following XML codes are equivalent.

+----------------------------------------------------------+-----------------------------------------------------------+
| **DataReader profile** - Definition method 1             | **DataReader profile** - Definition method 2              |
+----------------------------------------------------------+-----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               | .. literalinclude:: /../code/XMLTester.xml                |
|   :language: xml                                         |   :language: xml                                          |
|   :start-after: <!-->XML-DATAREADER-COMPARISON<-->       |   :start-after: <!-->XML-SUBSCRIBER-COMPARISON<-->        |
|   :end-before: <!--><-->                                 |   :end-before: <!--><-->                                  |
+----------------------------------------------------------+-----------------------------------------------------------+

.. important::

    The ``<subscriber>`` and ``<data_reader>`` XML tags are equivalent.
    Therefore, XML profiles in which the |DataReaders| are defined with the ``<subscriber>`` tag are fully compatible
    with Fast DDS.


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
     - Sets the name under which the ``<data_reader>`` profile is registered in the DDS Domain, |br|
       so that it can be loaded later by the |DomainParticipant|, as shown in |br|
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<data_reader>`` profile as the default profile.
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
     - It allows configuring some time related |br|
       parameters of the DataReader.
     - :ref:`Times <subtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. |br|
       It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. |br|
       It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<expectsInlineQos>``
     - It indicates if QoS is expected inline.
     - ``Boolean``
     - :class:`false`
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for DataReaders's |br| history.
     - :ref:`memorymanagementpolicy`
     - :class:`PREALLOCATED`
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for StaticEndpointDiscovery.
     - ``Int16``
     - -1
   * - ``<entityID>``
     - Set the |entity_id| of the |RTPSEndpointQos| |br| class.
     - ``Int16``
     - -1
   * - ``<matchedPublishersAllocation>``
     - Sets the limits of a DataWriters limited |br|
       collection, as well as the DataReader |br|
       :ref:`CommonAlloc`. See |br|
       :ref:`participantresourcelimitsqos`.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-SUBSCRIBER<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-58, 60-61

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _subtimes:

Times
""""""

+------------------------------+-------------------------------------------------------+---------------------+---------+
| Name                         | Description                                           | Values              | Default |
+==============================+=======================================================+=====================+=========+
| ``<initialAcknackDelay>``    | Initial ACKNACK delay.                                | :ref:`DurationType` | 70 ms   |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<heartbeatResponseDelay>`` | Heartbeat response time delay when receiving          | :ref:`DurationType` | 5 ms    |
|                              | an acknack.                                           |                     |         |
+------------------------------+-------------------------------------------------------+---------------------+---------+
