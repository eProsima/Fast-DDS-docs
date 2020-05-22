.. _subscriberprofiles:

Subscriber profiles
-------------------

Subscriber profiles allow declaring :ref:`Subscriber configuration <pubsubconfiguration>` from an XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

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

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<topic>``
     - :ref:`TopicType` configuration of the subscriber.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - Subscriber :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It allows configuring some time related parameters of the subscriber.
     - :ref:`Times <subtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<expectsInlineQos>``
     - It indicates if QOS is expected inline.
     - ``Boolean``
     - :class:`false`
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for subscriber's history.
     - :ref:`historyMemoryPolicy <mempol>`
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
     - EntityId of the *endpoint*.
     - ``Int16``
     - -1
   * - ``<matchedPublishersAllocation>``
     - Subscriber :ref:`CommonAlloc` related to the number of matched publishers.
     - :ref:`CommonAlloc`
     -

.. _subtimes:

**Times**

+------------------------------+----------------------------------+---------------------+---------+
| Name                         | Description                      | Values              | Default |
+==============================+==================================+=====================+=========+
| ``<initialAcknackDelay>``    | Initial AckNack delay.           | :ref:`DurationType` | ~45 ms  |
+------------------------------+----------------------------------+---------------------+---------+
| ``<heartbeatResponseDelay>`` | Delay to be applied when         | :ref:`DurationType` | ~4.5 ms |
|                              | a heartbeat message is received. |                     |         |
+------------------------------+----------------------------------+---------------------+---------+
