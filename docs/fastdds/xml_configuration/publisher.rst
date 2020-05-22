.. _publisherprofiles:

Publisher profiles
------------------

Publisher profiles allow declaring :ref:`Publisher configuration <pubsubconfiguration>` from an XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in the :ref:`loadingapplyingprofiles` section.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PUBLISHER<-->
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
     - :ref:`TopicType` configuration of the publisher.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - Publisher :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It allows configuring some time related parameters of the publisher.
     - :ref:`Times <pubtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<throughputController>``
     - Limits the output bandwidth of the publisher.
     - :ref:`Throughput`
     -
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for publisher's history.
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
   * - ``<matchedSubscribersAllocation>``
     - Publisher :ref:`CommonAlloc` related to the number of matched subscribers.
     - :ref:`CommonAlloc`
     -

.. _pubtimes:

**Times**

+------------------------------+-------------------------------+---------------------+---------+
| Name                         | Description                   | Values              | Default |
+==============================+===============================+=====================+=========+
| ``<initialHeartbeatDelay>``  | Initial heartbeat delay.      | :ref:`DurationType` | ~45 ms  |
+------------------------------+-------------------------------+---------------------+---------+
| ``<heartbeatPeriod>``        | Periodic HB period.           | :ref:`DurationType` | 3 s     |
+------------------------------+-------------------------------+---------------------+---------+
| ``<nackResponseDelay>``      | Delay to apply to the         | :ref:`DurationType` | ~45 ms  |
|                              | response of a ACKNACK         |                     |         |
|                              | message.                      |                     |         |
+------------------------------+-------------------------------+---------------------+---------+
| ``<nackSupressionDuration>`` | This time allows the          | :ref:`DurationType` | 0 ms    |
|                              | RTPSWriter to ignore          |                     |         |
|                              | nack messages too soon        |                     |         |
|                              | after the data has been sent. |                     |         |
+------------------------------+-------------------------------+---------------------+---------+
