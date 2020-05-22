.. _commonxml:

Common
------

In the above profiles, some types are used in several different places. To avoid too many details, some of that
places have a tag like :class:`LocatorListType` that indicates that field is defined in this section.

.. _LocatorListType:

LocatorListType
^^^^^^^^^^^^^^^

It represents a list of :class:`Locator_t`.
LocatorListType is normally used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that expect a list of locators and give it sense,
for example, in ``<defaultUnicastLocatorList>``.
The locator kind is defined by its own tag and can take the values ``<udpv4>``, ``<tcpv4>``, ``<udpv6>``, and
``<tcpv6>``:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LOCATOR-LIST<-->
    :end-before: <!--><-->

In this example, there are one locator of each kind in ``<defaultUnicastLocatorList>``.

Let's see each possible Locator's field in detail:

+---------------------+----------------------------------+----------------------------------+------------------+
| Name                | Description                      | Values                           | Default          |
+=====================+==================================+==================================+==================+
| ``<port>``          | RTPS port number of the locator. | ``Uint32``                       | 0                |
|                     | *Physical port* in UDP,          |                                  |                  |
|                     | *logical port* in TCP.           |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<physical_port>`` | TCP's *physical port*.           | ``Uint32``                       | 0                |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<address>``       | IP address of the locator.       | ``string`` with expected format  | ""               |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<unique_lan_id>`` | The LAN ID uniquely identifies   | ``string`` (16 bytes)            |                  |
|                     | the LAN the locator belongs to   |                                  |                  |
|                     | (**TCPv4 only**).                |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<wan_address>``   | WAN IPv4 address                 | ``string`` with IPv4 Format      | :class:`0.0.0.0` |
|                     | (**TCPv4 only**).                |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+


.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType (XML label ``<propertiesPolicy>``) allows defining a set of generic properties.
It's useful at defining extended or custom configuration parameters.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PROPERTIES-POLICY<-->
    :end-before: <!--><-->

+-----------------+-------------------------------------------+-------------+----------------+
| Name            | Description                               | Values      | Default        |
+=================+===========================================+=============+================+
| ``<name>``      | Name to identify the property.            | ``string``  |                |
+-----------------+-------------------------------------------+-------------+----------------+
| ``<value>``     | Property's value.                         | ``string``  |                |
+-----------------+-------------------------------------------+-------------+----------------+
| ``<propagate>`` | Indicates if it is going to be serialized | ``Boolean`` | :class:`false` |
|                 | along with the object it belongs to.      |             |                |
+-----------------+-------------------------------------------+-------------+----------------+

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time and it's commonly used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that give it sense, like ``<leaseAnnouncement>`` or
``<leaseDuration>``.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

Duration time can be defined through ``<sec>`` plus ``<nanosec>`` labels (see table below). An infinite value can be
specified by using the values :class:`DURATION_INFINITY`, :class:`DURATION_INFINITE_SEC` and
:class:`DURATION_INFINITE_NSEC`.

+----------------+-----------------------------------------------------------------+------------+---------+
| Name           | Description                                                     | Values     | Default |
+================+=================================================================+============+=========+
| ``<sec>``      | Number of seconds.                                              | ``Int32``  | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+
| ``<nanosec>``  | Number of nanoseconds.                                          | ``UInt32`` | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+

.. _TopicType:

Topic Type
^^^^^^^^^^

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange
messages.
There is a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->

+-------------------------+-------------------------------+-----------------------------------+-----------------+
| Name                    | Description                   | Values                            | Default         |
+=========================+===============================+===================================+=================+
| ``<kind>``              | It defines the Topic's        | :class:`NO_KEY`,                  | :class:`NO_KEY` |
|                         | kind                          | :class:`WITH_KEY`                 |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<name>``              | It defines the Topic's        | ``string_255``                    |                 |
|                         | name. Must be unique.         |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<dataType>``          | It references the             | ``string_255``                    |                 |
|                         | Topic's data type.            |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<historyQos>``        | It controls the behavior      | :ref:`HistoryQos <hQos>`          |                 |
|                         | of *Fast RTPS* when the value |                                   |                 |
|                         | of an instance changes before |                                   |                 |
|                         | it is finally communicated to |                                   |                 |
|                         | some of its existing          |                                   |                 |
|                         | DataReader entities.          |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<resourceLimitsQos>`` | It controls the resources     | :ref:`ResourceLimitsQos <rLsQos>` |                 |
|                         | that *Fast RTPS* can use      |                                   |                 |
|                         | in order to meet the          |                                   |                 |
|                         | requirements imposed          |                                   |                 |
|                         | by the application            |                                   |                 |
|                         | and other QoS settings.       |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+

.. _hQos:

**HistoryQoS**

It controls the behavior of *Fast RTPS* when the value of an instance changes before it is finally
communicated to some of its existing DataReader entities.

+-------------+------------------------+---------------------+--------------------+
| Name        | Description            | Values              | Default            |
+=============+========================+=====================+====================+
| ``<kind>``  | See description below. | :class:`KEEP_LAST`, | :class:`KEEP_LAST` |
|             |                        | :class:`KEEP_ALL`   |                    |
+-------------+------------------------+---------------------+--------------------+
| ``<depth>`` |                        | ``UInt32``          | 1000               |
+-------------+------------------------+---------------------+--------------------+

| If the ``<kind>`` is set to :class:`KEEP_LAST`, then *Fast RTPS* will only attempt to keep the latest values of the
  instance and discard the older ones.
| If the ``<kind>`` is set to :class:`KEEP_ALL`, then *Fast RTPS* will attempt to maintain and deliver all the values
  of the instance to existing subscribers.
| The setting of ``<depth>`` must be consistent with the :ref:`ResourceLimitsQos <rLsQos>`
  ``<max_samples_per_instance>``.
  For these two QoS to be consistent, they must verify that ``depth <= max_samples_per_instance``.

.. _rLsQos:

**ResourceLimitsQos**

It controls the resources that *Fast RTPS* can use in order to meet the requirements imposed by the
application and other QoS settings.

+--------------------------------+---------------------------------------------------------+------------+---------+
| Name                           | Description                                             | Values     | Default |
+================================+=========================================================+============+=========+
| ``<max_samples>``              | It must verify that                                     | ``UInt32`` | 5000    |
|                                | ``max_samples >= max_samples_per_instance``.            |            |         |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<max_instances>``            | It defines the maximum number of instances.             | ``UInt32`` | 10      |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<max_samples_per_instance>`` | It must verify that :ref:`HistoryQos <hQos>`            | ``UInt32`` | 400     |
|                                | ``depth <= max_samples_per_instance``.                  |            |         |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<allocated_samples>``        | It controls the maximum number of samples to be stored. | ``UInt32`` | 100     |
+--------------------------------+---------------------------------------------------------+------------+---------+

.. _CommonQOS:

QOS
^^^

The quality of service (QoS) handles the restrictions applied to the application.

.. AFTER DURABILITY
    <durabilityService>
        <!-- DURABILITY_SERVICE -->
    </durabilityService>

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-QOS<-->
    :end-before: <!--><-->

+--------------------------+----------------------------------+-------------------------------+------------------------+
| Name                     | Description                      | Values                        | Default                |
+==========================+==================================+===============================+========================+
| ``<durability>``         | It is defined in                 |:class:`VOLATILE`,             | :class:`VOLATILE`      |
|                          | :ref:`SettingDataDurability`     |:class:`TRANSIENT_LOCAL`       |                        |
|                          | section.                         |:class:`TRANSIENT`             |                        |
|                          |                                  |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<liveliness>``         | Defines the liveliness of the    | :ref:`liveliness-qos`         |                        |
|                          | publisher.                       |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<reliability>``        | It is defined in                 | :class:`RELIABLE`,            | :class:`RELIABLE`      |
|                          | :ref:`reliability` section.      | :class:`BEST_EFFORT`          |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<partition>``          | It allows the introduction of    |                               | ``List <string>``      |
|                          | a logical partition concept      |                               |                        |
|                          | inside the `physical` partition  |                               |                        |
|                          | induced by a domain.             |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<deadline>``           | It is defined in                 |                               |                        |
|                          | :ref:`deadline-qos`              | Deadline period as a          | :class:`c_TimeInfinite`|
|                          | section.                         | :ref:`DurationType`           |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<lifespan>``           | It is defined in                 | Lifespan duration as a        | :class:`c_TimeInfinite`|
|                          | :ref:`lifespan-qos` section.     | :ref:`DurationType`           |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<disablePositiveAcks>``| It is defined in                 |                               | It is disabled by      |
|                          | section                          |                               | default and            |
|                          | :ref:`disable-positive-acks-qos` |                               | ``duration`` is set    |
|                          |                                  |                               | to                     |
|                          |                                  |                               | :class:`c_TimeInfinite`|
+--------------------------+----------------------------------+-------------------------------+------------------------+

..
    .. note::

        - :class:`DURATION` means it expects a :ref:`DurationType`.

    ..  - :class:`DURABILITY_SERVICE` means that the label is a :ref:`DurabilityServiceType` block.::

        - :class:`LIVELINESS` means that the label is a :ref:`LiveLinessType` block.


    .. NOT YET SUPPORTED
        - ``<latencyBudget>``: Latency budget os the samples as :ref:`DurationType` within a ``<duration>`` tag.

        - ``<userData>``: Allows adding custom information.

        - ``<timeBasedFilter>``: Allows filtering by time. It's a :ref:`DurationType` within a ``<minimum_separation>`` tag.

        - ``<ownership>``: ``<kind>`` determines whether an instance of the Topic is owned by a single Publisher. If the selected ownership is :class:`EXCLUSIVE` the Publisher will use the Ownership strength value as the strength of its publication. Only the publisher with the highest strength can publish in the same Topic with the same Key.

        - ``<destinationOrder>``: ``<kind>`` determines the destination timestamp. :class:`BY_RECEPTION_TIMESTAMP` for reception and :class:`BY_SOURCE_TIMESTAMP` for the source.

        - ``<presentation>``:
            * ``<access_scope>`` defines the scope of presentation and can be :class:`INSTANCE`, :class:`TOPIC`, or :class:`GROUP`.

            * ``<coherent_access>`` Boolean value to set if the access must be coherent.

            * ``<ordered_access>`` Boolean value to set if the access must be ordered.

        - ``<topicData>``: Allows adding custom topic data.

        - ``<groupData>``: Allows adding custom group data.



.. .. _DurabilityServiceType:

    DurabilityServiceType
    ^^^^^^^^^^^^^^^^^^^^^

    Durability defines the behavior regarding samples that existed on the topic before a subscriber joins.

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_DURABILITYSERVICE<-->
       :end-before: <!--><-->

    - ``<history_kind>``: History handling kind. It accepts :class:`KEEP_LAST` and :class:`KEEP_ALL` values.

    - ``<history_depth>``: Allows establishing the depth of the history.

    - ``<max_samples>``: The maximum number of samples to be stored.

    - ``<max_instances>``: The maximum number of history instances.

    - ``<max_samples_per_instance>``: Allows establishing the maximum number of samples per history instance.

.. _Throughput:

Throughput Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Throughput Configuration allows to limit the output bandwidth.

+-----------------------+-----------------------------------------------------------+------------+------------+
| Name                  | Description                                               | Values     | Default    |
+=======================+===========================================================+============+============+
| ``<bytesPerPeriod>``  | Packet size in bytes that this controller will allow in   | ``UInt32`` | 4294967295 |
|                       | a given period.                                           |            |            |
+-----------------------+-----------------------------------------------------------+------------+------------+
| ``<periodMillisecs>`` | Window of time in which no more than ``<bytesPerPeriod>`` | ``UInt32`` | 0          |
|                       | bytes are allowed.                                        |            |            |
+-----------------------+-----------------------------------------------------------+------------+------------+

.. _CommonAlloc:

Allocation Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Allocation Configuration allows to control the allocation behavior of internal collections for which the number
of elements depends on the number of entities in the system.

For instance, there are collections inside a publisher which depend on the number of subscribers matching with it.

See :ref:`realtime-allocations` for detailed information on how to tune allocation related parameters.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<initial>``
     - Number of elements for which space is initially allocated.
     - ``UInt32``
     - 0
   * - ``<maximum>``
     - Maximum number of elements for which space will be allocated.
     - ``UInt32``
     - 0 (means no limit)
   * - ``<increment>``
     - Number of new elements that will be allocated when more space is necessary.
     - ``UInt32``
     - 1

.. _MessageMaxSize:

Submessage Size Limit
^^^^^^^^^^^^^^^^^^^^^

While some submessages have a fixed size (for example, SequenceNumber), others have a variable size depending on the
data they contain.
Processing a submessage requires having a memory chunk large enough to contain a copy of the submessage data.
That is easy to handle when dealing with fixed variable submessages, as size is known and memory can be allocated
beforehand.
For variable size submessages on the other hand, two different strategies can be used:

    - Set a maximum size for the data container, which will be allocated beforehand during the participant's setup.
      This avoids dynamic allocations during message communication.
      However, any submessages with a larger payload than the
      defined maximum will not fit in, and will therefore be discarded.
    - Do not set any maximum size for the data container, and instead allocate the required memory dynamically upon
      submessage arrival (according to the size declared on the submessage header).
      This allows for any size of submessages, at the cost of dynamic allocations during message decoding.

.. _mempol:

History Memory Policy Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Controls the allocation behavior of the change histories.

* **PREALLOCATED**: As the history gets larger, memory is allocated in chunks. Each chunk accommodates
  a number of changes, and no more allocations are done until that chunk is full. Provides minimum
  number of dynamic allocations at the cost of increased memory footprint. Maximum payload size of
  changes must be appropriately configured, as history will not be able to accommodate changes
  with larger payload after the allocation.
* **PREALLOCATED_WITH_REALLOC**: Like PREALLOCATED, but preallocated memory can be reallocated
  to accommodate changes with larger payloads than the defined maximum.
* **DYNAMIC**: Every change gets a fresh new allocated memory of the correct size.
  It minimizes the memory footprint, at the cost of increased number of dynamic allocations.
* **DYNAMIC_REUSABLE**: Like DYNAMIC, but instead of deallocating the memory when the change is removed
  from the history, it is reused for a future change, reducing the amount of dynamic allocations.
  If the new change has larger payload, it will be reallocated to accommodate the new size.
