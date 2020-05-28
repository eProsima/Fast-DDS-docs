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

.. Note::

    The ``<subscriber>`` and ``<data_reader>`` XML tags are equivalent.
    Therefore, **XML profiles in which the |DataReaders| are defined with the ``<subscriber>`` tag are fully compatible
    with Fast DDS**.

The ``<data_reader>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.
The mandatory ``profile_name`` attribute is the name under which the ``<data_reader>`` profile is registered in the DDS
Domain, so that it can be loaded later by a |DomainParticipant|, as shown in :ref:`loadingapplyingprofiles`.
The second attribute, ``is_default_profile``, is an optional attribute and sets the ``<data_reader>`` profile as the
default profile.

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
     - It allows configuring some time related parameters |br| of the DataReader.
     - :ref:`Times <subtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. |br| It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. |br| It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<expectsInlineQos>``
     - It indicates if QoS is expected inline.
     - ``Boolean``
     - :class:`false`
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for subscriber's history.
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
     - Subscriber :ref:`CommonAlloc` |br| related to the number of matched DataWriters.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-SUBSCRIBER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _subtimes:

Times
^^^^^

+------------------------------+-------------------------------------------------------+---------------------+---------+
| Name                         | Description                                           | Values              | Default |
+==============================+=======================================================+=====================+=========+
| ``<initialAcknackDelay>``    | Initial acknack delay.                                | :ref:`DurationType` | ~45 ms  |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<heartbeatResponseDelay>`` | Heartbeat response time delay when receiving          | :ref:`DurationType` | ~4.5 ms |
|                              | an acknack.                                           |                     |         |
+------------------------------+-------------------------------------------------------+---------------------+---------+
