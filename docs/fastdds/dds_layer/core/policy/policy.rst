.. role:: raw-html(raw)
    :format: html

.. _dds_layer_core_policy:

Policy
======

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to  to define how each
entity will behave.
To increase the flexibility of the system, the QoS is decomposed in several QoS Policies that can be configured
independently.
However, there may be cases where several policies conflict.

There are QoS Policies that are immutable, which means that only can be specified either at the entity creation or
before calling the enable operation.

Each DDS Entity has a specific set of QoS Policies that can be a mix of :ref:`standard <standard>` and
:ref:`extensions <extensions>`.

.. _standard:

Standard QoS Policies
---------------------

This section explains each of the DDS standard QoS Policies:

.. _deadlineqospolicy:

DeadlineQosPolicy
^^^^^^^^^^^^^^^^^

This QoS policy raises an alarm when the frequency of new samples falls below a certain threshold.
It is useful for cases where data is expected to be updated periodically.

On the publishing side, the deadline defines the maximum period in which the application is expected to supply a new
sample.
On the subscribing side, it defines the maximum period in which new samples should be received.

For topics with keys, this QoS is applied by key. Imagine for example we are publishing vehicle positions, and we want
to enforce a position of each vehicle is published periodically, in that case, we can set the ID of the vehicle as the
key of the topic, and use the deadline QoS.

List of QoS Policy data members:

+--------------------------+--------------------------------------------------+-----------------------------------+
| Data Member Name         | Type                                             | Default Value                     |
+==========================+==================================================+===================================+
| period                   | fastrtps::Duration_t                             | c_TimeInfinite                    |
+--------------------------+--------------------------------------------------+-----------------------------------+

.. note::

   This QoS Policy concerns to Topic, DataReader and DataWriter entities.
   :raw-html:`<br />`
   It can be changed after the creation of the entities.

.. warning::

    For DataWriters and DataReaders to match, the offered deadline period must be less than or equal to the requested deadline
    period, otherwise, the entities are considered to be incompatible.
    :raw-html:`<br />`
    The DeadlineQosPolicy must be set consistently with the :ref:`timebasedfilterqospolicy`, which means that the deadline period
    must be higher or equal to the minimum separation.

.. _deadline_example:

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DEADLINE_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: ../../../../../code/XMLTester.xml
   :language: xml
   :start-after: <!-->XML_DEADLINE
   :end-before: <!--><-->

.. _destinationorderqospolicy:

DestinationOrderQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

Multiple DataWriters can send messages in the same Topic using the same key, and on the DataReader side all those
messages are stored within the same instance of data.
This QoS policy controls the criteria used to determine the logical order of those messages.
The behavior of the system depends on the value of the :ref:`destinationorderqospolicykind`.

List of QoS Policy data members:

+--------------------------+--------------------------------------------------+-----------------------------------+
| Data Member Name         | Type                                             | Default Value                     |
+==========================+==================================================+===================================+
| kind                     | :ref:`destinationorderqospolicykind`             | BY_RECEPTION_TIMESTAMP            |
+--------------------------+--------------------------------------------------+-----------------------------------+

.. note::
    This QoS Policy concerns to Topic, DataReader and DataWriter entities.
    :raw-html:`<br />`
    It cannot be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`destinationorder_compatibilityrule` for
    further details.

.. _destinationorderqospolicykind:

DestinationOrderQosPolicyKind
"""""""""""""""""""""""""""""

There are two possible values:

* **By reception timestamp**: This indicates that the data is ordered based on the reception time at each DataReader,
  which means that the last received value should be the one kept.
  This option may cause that each DataReader ends up with a different final value, since the DataReaders may receive
  the data at different times.
* **By source timestamp**: This indicates that the data is ordered based on the DataWriter timestamp at the time the
  message is sent.
  This option guarantees the consistency of the final value.

Both options depend on the values of the :ref:`ownershipqospolicy` and :ref:`ownershipstrengthqospolicy`, meaning that
if the Ownership is set to EXCLUSIVE and the last value came from a DataWriter with low ownership strength, it will be
discarded.

.. _destinationorder_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DestinationOrderQosPolicy in DataReaders and DataWriters when they have different
kind values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is::

 BY_RECEPTION_TIMESTAMP < BY_SOURCE_TIMESTAMP

Table with the possible combinations:

+------------------------+-------------------------+-----------------+
| DataWriter kind        | DataReader kind         | Compatibility   |
+========================+=========================+=================+
| By Reception Timestamp | By Reception Timestamp  | Yes             |
+------------------------+-------------------------+-----------------+
| By Reception Timestamp | By Source Timestamp     | No              |
+------------------------+-------------------------+-----------------+
| By Source Timestamp    | By Reception Timestamp  | Yes             |
+------------------------+-------------------------+-----------------+
| By Source Timestamp    | By Source Timestamp     | Yes             |
+------------------------+-------------------------+-----------------+

.. _durabilityqospolicy:

DurabilityQosPolicy
^^^^^^^^^^^^^^^^^^^

A DataWriter can send messages throughout a Topic even if there are no DataReaders on the network.
Moreover, a DataReader that joins to the Topic after some data has been written could be interested in accessing
that information.

The Durability QoS Policy defines how the system will behave regarding those samples that existed on the Topic before
the DataReader joins.
The behavior of the system depends on the value of the :ref:`DurabilityQosPolicyKind<durabilitykind>`.

List of QoS Policy data members:

+--------------------------+--------------------------------------------------+-----------------------------------+
| Data Member Name         | Type                                             | Default Value                     |
+==========================+==================================================+===================================+
|                          |                                                  | VOLATILE for DataReaders          |
| kind                     | :ref:`DurabilityQosPolicyKind_t<durabilitykind>` | :raw-html:`<br />`                |
|                          |                                                  | TRANSIENT LOCAL for DataWriters   |
+--------------------------+--------------------------------------------------+-----------------------------------+

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`durability_compatibilityrule` for
    further details.

.. _durabilitykind:

DurabilityQosPolicyKind
"""""""""""""""""""""""

There are four possible values:

* **Volatile**: Past samples are ignored and a joining DataReader receives samples generated after the moment it
  matches.
* **Transient Local**: When a new DataReader joins, its History is filled with past samples.
* **Transient**: When a new DataReader joins, its History is filled with past samples, which are stored on persistent
  storage (see :ref:`persistence`).
* **Persistent** (`Not Implemented`): All the sample are stored on a permanent storage, so that they can outlive a
  system session.

.. _durability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DurabilityQosPolicy in DataReaders and DataWriters when they have different kind
values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is::

 VOLATILE < TRANSIENT LOCAL < TRANSIENT < PERSISTENT

Table with the possible combinations:

+--------------------+-------------------+-----------------+
| DataWriter kind    | DataReader kind   | Compatibility   |
+====================+===================+=================+
| Volatile           | Volatile          | Yes             |
+--------------------+-------------------+-----------------+
| Volatile           | Transient Local   | No              |
+--------------------+-------------------+-----------------+
| Volatile           | Transient         | No              |
+--------------------+-------------------+-----------------+
| Transient Local    | Volatile          | Yes             |
+--------------------+-------------------+-----------------+
| Transient Local    | Transient Local   | Yes             |
+--------------------+-------------------+-----------------+
| Transient Local    | Transient         | No              |
+--------------------+-------------------+-----------------+
| Transient          | Volatile          | Yes             |
+--------------------+-------------------+-----------------+
| Transient          | Transient Local   | Yes             |
+--------------------+-------------------+-----------------+
| Transient          | Transient         | Yes             |
+--------------------+-------------------+-----------------+

.. _durability_example:

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DURABILITY_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: ../../../../../code/XMLTester.xml
   :language: xml
   :start-after: <!-->PUBSUB_API_CONF_PUBSUB_DURABILITY
   :end-before: <!--><-->

.. _durabilityserviceqospolicy:

DurabilityServiceQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy is used to configure the :ref:`historyqospolicy` and :ref:`resourcelimitsqospolicy` of the fictitious
DataReader and DataWriter used when the :ref:`durabilityqospolicy` kind is set to TRANSIENT or PERSISTENT.

Those entities are used to simulate the persistent storage. The fictitious DataReader reads the data written on the
Topic and stores it, so that if the user DataWriter doesn't have the information requested by the user DataReaders, the
fictitious DataWriter takes care of sending that information.

List of QoS Policy data members:

+--------------------------+-----------------------------+-----------------------+
| Data Member Name         | Type                        | Default Value         |
+==========================+=============================+=======================+
| service_cleanup_delay    | fastrtps::Duration_t        | c_TimeZero            |
+--------------------------+-----------------------------+-----------------------+
| history_kind             | :ref:`historyqospolicykind` | KEEP_LAST             |
+--------------------------+-----------------------------+-----------------------+
| history_depth            | int32_t                     | 1                     |
+--------------------------+-----------------------------+-----------------------+
| max_samples              | int32_t                     | -1 (Length Unlimited) |
+--------------------------+-----------------------------+-----------------------+
| max_instances            | int32_t                     | -1 (Length Unlimited) |
+--------------------------+-----------------------------+-----------------------+
| max_samples_per_instance | int32_t                     | -1 (Length Unlimited) |
+--------------------------+-----------------------------+-----------------------+

* **Service cleanup delay**: It controls when the service can remove all the information regarding a data instance. That
  information is kept until the following conditions are met:

  * The instance has been explicitly disposed and its InstanceState becomes `NOT_ALIVE_DISPOSED`.
  * There isn't any alive DataWriter writing the instance, which means that all existing writers either unregister the
    instance or lose their liveliness.
  * A time interval longer than the one established on the service_cleanup_delay has elapsed since the moment the
    service detected that the two previous conditions were met.

* **History kind**: Controls the kind of the :ref:`historyqospolicy` associated with the Durability Service fictitious
  entities.
* **History depth**: Controls the depth of the :ref:`historyqospolicy` associated with the Durability Service fictitious
  entities.
* **Max samples**: Controls the maximum number of samples of the :ref:`resourcelimitsqospolicy` associated with the
  Durability Service fictitious entities.
  This value must be higher than the maximum number of samples per instance.
* **Max instances**: Controls the maximum number of instances of the :ref:`resourcelimitsqospolicy` associated with the
  Durability Service fictitious entities.
* **Max samples per instance**: Controls the maximum number of samples within an instance of the
  :ref:`resourcelimitsqospolicy` associated with the Durability Service fictitious entities.
  This value must be lower than the maximum number of samples.

.. note::
     This QoS Policy concerns to Topic and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. _entityfactoryqospolicy:

EntityFactoryQosPolicy
^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy controls the behavior of an :ref:`dds_layer_core_entity` when it acts as a factory for other entities.
As you know, the entities can be created either enabled or disabled, a fact that can be controlled using this QoS
Policy.
By default, all the entities are created enabled, but if you change the value of the `autoenable_created_entities` to
false, the new entities will be created disabled.

List of QoS Policy data members:

+-----------------------------+-------------------------------+-------------------+
| Data Member Name            | Type                          | Default Value     |
+=============================+===============================+===================+
| autoenable_created_entities | bool                          | true              |
+-----------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to DomainParticipantFactory (as factory for DomainParticipant), DomainParticipant (as factory for
     Publisher, Subscriber and Topic), Publisher (as factory for DataWriter) and Subscriber (as factory for DataReader).
     :raw-html:`<br />`
     It can be changed after the creation of the entities, but it only affects those entities created after the change.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_ENTITY_FACTORY_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _groupqospolicy:

GroupDataQosPolicy
^^^^^^^^^^^^^^^^^^

Allow the application to attach additional information to created Publishers or Subscribers.
This data is common to all DataWriters\\DataReaders belonging to the Publisher\\Subscriber and it is propagated by means
of the built-in topics.

This QoS Policy can be used in combination with DataWriter and DataReader listeners to implement a matching policy
similar to the :ref:`PartitionQosPolicy <partitionqospolicy>`.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| collection               | std::vector<fastrtps::octet>  | Empty vector      |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Publisher and Subscriber entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_GROUP_DATA_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _historyqospolicy:

HistoryQosPolicy
^^^^^^^^^^^^^^^^

This QoS Policy controls the behavior of the system when the value of an instance changes one or more times before it
can be successfully communicated to the existing DataReader entities.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| kind                     | :ref:`historyqospolicykind`   | KEEP_LAST         |
+--------------------------+-------------------------------+-------------------+
| depth                    | int32_t                       | 1                 |
+--------------------------+-------------------------------+-------------------+

* **kind**: Controls if the service should deliver only the most recent values, all the intermediate values or do
  something in between.
  See :ref:`historyqospolicykind` for further details.
* **depth**: Establishes the maximum number of samples that must be kept on the history.
  It only have effect if the kind is set to KEEP_LAST and it need to be consistent with the
  :ref:`resourcelimitsqospolicy`, which means that its value must be lower or equal to max_samples_per_instance.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. _historyqospolicykind:

HistoryQosPolicyKind
""""""""""""""""""""
There are two possible values:

* **Keep Last**: The service will only attempt to keep the most recent values of the instance and discard the older
  ones.
  The maximum number of samples to keep and deliver is defined by the `depth` of the HistoryQosPolicy, which needs to
  be consistent with the :ref:`resourcelimitsqospolicy` settings.
* **Keep All**: The service will attempt to keep all the values of the instance until it can be delivered to all the
  existing Subscribers.
  If this option is selected, the depth will not have any effect, so the history is only limited by the values set in
  :ref:`resourcelimitsqospolicy`.
  If the limit is reached, the behavior of the system depends on the :ref:`reliabilityqospolicy`, if its kind is
  BEST_EFFORT the older values will be discarded, but if it is RELIABLE the service blocks the DataWriter until the old
  values are delivered to all existing Subscribers.


Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_HISTORY_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-HISTORY<-->
    :end-before: <!--><-->

.. _latencybudgetqospolicy:

LatencyBudgetQosPolicy
^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy urgency of the data communication, specifying the maximum acceptable delay from the time the data is
written until the data is inserted on the cache of the receiver application and notified of the fact.
That delay by default is set to 0 in order to optimize the internal operations.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| duration                 | fastrtps::Duration_t          | c_TimeZero        |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`latencybudget_compatibilityrule`
    for further details.

.. _latencybudget_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between LatencyBudgetQosPolicy in DataReaders and DataWriters, the DataWriter duration
must be lower or equal to the DataReader duration.

.. _lifespanqospolicy:

LifespanQosPolicy
^^^^^^^^^^^^^^^^^

Each data sample written by a DataWriter has an associated expiration time beyond which the data is removed from the
DataWriter and DataReader history as well as from the transient and persistent information caches.

By default, the `duration` is infinite, which means that there isn't a maximum duration for the validity of the samples
written by the DataWriter.

The expiration time is computed by adding the `duration` to the source timestamp, which can be calculated automatically
if :func:`write` member function is called or supplied by the application by means of :func:`write_w_timestamp` member
function.
The DataReader is allowed to use the reception timestamp instead of the source timestamp.


List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| duration                 | fastrtps::Duration_t          | c_TimeInfinite    |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Topic and DataWriter entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_LIFESPAN_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_LIFESPAN
    :end-before: <!--><-->

.. _livelinessqospolicy:

LivelinessQosPolicy
^^^^^^^^^^^^^^^^^^^

This QoS Policy controls the mechanism used by the service to ensure that a particular entity on the network is still
alive.
There are different settings that allow distinguishing between applications where data is updated periodically and
applications where data is changed sporadically.
It also allows customizing the application regarding the kind of failures that should be detected by the liveliness
mechanism.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| kind                     | :ref:`livelinessqospolicykind`| AUTOMATIC         |
+--------------------------+-------------------------------+-------------------+
| lease_duration           | fastrtps::Duration_t          | c_TimeInfinite    |
+--------------------------+-------------------------------+-------------------+
| announcement_period      | fastrtps::Duration_t          | c_TimeInfinite    |
+--------------------------+-------------------------------+-------------------+

* **Kind**: This data member establishes if the service needs to assert the liveliness automatically or if it needs
  to wait until the liveliness is asserted by the publishing side.
  See :ref:`livelinessqospolicykind` for further details.
* **Lease Duration**: Amount of time to wait since the last time the DataWriter asserts its liveliness to consider
  that it is no longer alive.
* **Announcement Period**: Amount of time between consecutive liveliness messages sent by the DataWriter.
  This data member only takes effect if the kind is AUTOMATIC or MANUAL_BY_PARTICIPANT and needs to be lower than the
  `lease_duration`.

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`liveliness_compatibilityrule`
    for further details.

.. _livelinessqospolicykind:

LivelinessQosPolicyKind
"""""""""""""""""""""""

There are three possible values:

* **Automatic**: The service takes the responsibility for renewing the leases at the required rates, as long as the
  local process where the participant is running and the link connecting it to remote participants exists, the
  entities within the remote participant will be considered alive.
  This kind is suitable for applications that only need to detect whether a remote application is still running.
* The two **Manual** modes require that the application on the publishing side asserts the liveliness periodically
  before the lease_duration timer expires. This action can be done by calling `assert_liveliness` member function, or
  by writing some data.

  * **Manual by Participant**: If one of the entities in the publishing side asserts its' liveliness, the service
    deduces that all other entities within the same DomainParticipant are also alive.
  * **Manual by Topic**: This mode is more restrictive and requires that at least one instance within the DataWriter
    is asserted to consider that the DataWriter is alive.

.. _liveliness_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between LivelinessQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
higher or equal to the DataReader kind.
And the order between the different kinds is::

 AUTOMATIC < MANUAL BY PARTICIPANT < MANUAL BY TOPIC

Table with the possible combinations:

+-----------------------+-----------------------+-----------------+
| DataWriter kind       | DataReader kind       | Compatibility   |
+=======================+=======================+=================+
| Automatic             | Automatic             | Yes             |
+-----------------------+-----------------------+-----------------+
| Automatic             | Manual By Participant | No              |
+-----------------------+-----------------------+-----------------+
| Automatic             | Manual By Topic       | No              |
+-----------------------+-----------------------+-----------------+
| Manual By Participant | Automatic             | Yes             |
+-----------------------+-----------------------+-----------------+
| Manual By Participant | Manual By Participant | Yes             |
+-----------------------+-----------------------+-----------------+
| Manual By Participant | Manual By Topic       | No              |
+-----------------------+-----------------------+-----------------+
| Manual By Topic       | Automatic             | Yes             |
+-----------------------+-----------------------+-----------------+
| Manual By Topic       | Manual By Participant | Yes             |
+-----------------------+-----------------------+-----------------+
| Manual By Topic       | Manual By Topic       | Yes             |
+-----------------------+-----------------------+-----------------+

Additionally, the `lease_duration` of the DataWriter must also be greater than the `lease_duration` of the DataReader.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_LIVELINESS_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_LIVELINESS
    :end-before: <!--><-->

.. _ownershipqospolicy:

OwnershipQosPolicy
^^^^^^^^^^^^^^^^^^

This QoS Policy specifies whether it is allowed for multiple DataWriters to update the same instance of data, and if
so, how these modifications should be arbitrated.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| kind                     | :ref:`ownershipqospolicykind` | SHARED            |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`ownership_compatibilityrule`
    for further details.

.. _ownershipqospolicykind:

OwnershipQosPolicyKind
""""""""""""""""""""""

There are two possible values:

* **Shared**: This option indicates that the service doesn't enforce unique ownership for each instance.
  In this case, multiple DataWriters are allowed to update the same data instance and all the updates are made
  available to the existing DataReaders.
  Those updates are also subject to the :ref:`timebasedfilterqospolicy` or :ref:`historyqospolicy` settings, so they
  can be filtered.
* **Exclusive**: This option indicates that each instance can only be updated by one DataWriter, meaning that at any
  point in time a single DataWriter owns each instance and is the only one whose modifications will be visible for the
  existing DataWriters.
  The owner can be changed dynamically according to the highest `strength` between the alive DataWriters, which has not
  violated the deadline contract concerning the data instances.
  That `strength` can be changed using the :ref:`ownershipstrengthqospolicy`.

.. _ownership_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between OwnershipQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
equal to the DataReader kind.

Table with the possible combinations:

+-----------------------+-----------------------+-----------------+
| DataWriter kind       | DataReader kind       | Compatibility   |
+=======================+=======================+=================+
| Shared                | Shared                | Yes             |
+-----------------------+-----------------------+-----------------+
| Shared                | Exclusive             | No              |
+-----------------------+-----------------------+-----------------+
| Exclusive             | Shared                | No              |
+-----------------------+-----------------------+-----------------+
| Exclusive             | Exclusive             | Yes             |
+-----------------------+-----------------------+-----------------+

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_OWNERSHIP_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _ownershipstrengthqospolicy:

OwnershipStrengthQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy specifies the value of the `strength` used to arbitrate among multiple DataWriters that attempt to
modify the same data instance. It is only applicable if the :ref:`ownershipqospolicy` kind is set to EXCLUSIVE.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| value                    | uint32_t                      | 0                 |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_OWNERSHIP_STRENGTH_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _partitionqospolicy:

PartitionQosPolicy
^^^^^^^^^^^^^^^^^^

This Qos Policy allows the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match, but also they have to be on
the same logical partition.

The empty string is also considered as a valid partition and it matches with other partition names using the same rules
of string matching and regular-expression matching used for any other partition name.

List of QoS Policy data members:

+--------------------------+--------------------------------------+---------------------+
| Data Member Name         | Type                                 | Default Value       |
+==========================+======================================+=====================+
| max_size                 | uint32_t                             | 0 (Length Unlimited)|
+--------------------------+--------------------------------------+---------------------+
| partitions               | fastrtps::rtps::Serialized_Payload_t | Empty List          |
+--------------------------+--------------------------------------+---------------------+

* **Max Size**: Maximum size for the list of partition names.
* **Partitions**: List of partition names.

.. note::
     This QoS Policy concerns to Publisher and Subscriber entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PARTITION_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTITION
    :end-before: <!--><-->

.. _presentationqospolicy:

PresentationQosPolicy
^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies how the samples representing changes to data instances are presented to the subscribing
application.
It controls the extent to which changes to data instances can be made dependent on each other, as well as the kind
of dependencies that can be propagated and maintained.

List of QoS Policy data members:

+--------------------------+---------------------------------------------+---------------------+
| Data Member Name         | Type                                        | Default Value       |
+==========================+=============================================+=====================+
| access_scope             | :ref:`presentationqospolicyaccessscopekind` | INSTANCE            |
+--------------------------+---------------------------------------------+---------------------+
| coherent_access          | bool                                        | false               |
+--------------------------+---------------------------------------------+---------------------+
| ordered_access           | bool                                        | false               |
+--------------------------+---------------------------------------------+---------------------+

* **Access Scope**: Determines the largest scope spanning the entities for which the order and coherency can be
  preserved.
  See :ref:`presentationqospolicyaccessscopekind` for further details.
* **Coherent Access**: Controls whether the service will preserve grouping of changes made on the publishing side,
  such that they are received as a unit on the subscribing side.
* **Ordered Access**: Controls whether the service supports the ability of the subscriber to see changes in the same
  order as they occurred on the publishing side.

.. note::
     This QoS Policy concerns to Publisher and Subscriber entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`presentation_compatibilityrule`
    for further details.

.. _presentationqospolicyaccessscopekind:

PresentationQosPolicyAccessScopeKind
""""""""""""""""""""""""""""""""""""

There are three possible values, which have different behaviors depending on the values of coherent_access and
ordered_access variables:

* **Instance**: The changes to a data instance do not need to be coherent nor ordered with respect to the changes to
  any other instance, which means that the order and coherent changes apply to each instance separately.

  * Enabling the `coherent_access`, in this case, has no effect on how the subscriber can access the data as the scope
    is limited to each instance, changes to separate instances are considered independent and thus cannot be grouped
    by a coherent change.
  * Enabling the `ordered_access`, in this case, only affects to the changes within the same instance.
    Therefore, the changes made to two instances are not necessarily seen in the order they occur even if the same
    application thread and DataWriter made them.

* **Topic**: The scope spans to all the instances within the same DataWriter.

  * Enabling the `coherent_access` makes that the grouping made with changes within the same DataWriter will be
    available as coherent with respect to other changes to instances in that DataWriter, but will not be grouped with
    changes made to instances belonging to different DataWriters.
  * Enabling the `ordered_access` means that the changes made by a single DataWriter are made available to the
    subscribers in the same order that they occur, but the changes made to instances through different DataWriters are
    not necessarily seen in order.

* **Group**: The scope spans to all the instances belonging to DataWriters within the same Publisher.

  * Enabling the `coherent_access`, means that the coherent changes made to instances through DataWriters attached to a
    common Publisher are made available as a unit to remote subscribers.
  * Enabling the `ordered_access` with this scope makes that the changes done by any of the DataWriters attached to the
    same Publisher are made available to the subscribers in the same order they occur.

.. _presentation_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between PresentationQosPolicy in DataReaders and DataWriters, the Publisher access_scope
must be higher or equal to the Subscriber access_scope.
And the order between the different access scopes is::

 INSTANCE < TOPIC < GROUP

Table with the possible combinations:

+-----------------------+-----------------------+-----------------+
| Publisher scope       | Subscriber scope      | Compatibility   |
+=======================+=======================+=================+
| Instance              | Instance              | Yes             |
+-----------------------+-----------------------+-----------------+
| Instance              | Topic                 | No              |
+-----------------------+-----------------------+-----------------+
| Instance              | Group                 | No              |
+-----------------------+-----------------------+-----------------+
| Topic                 | Instance              | Yes             |
+-----------------------+-----------------------+-----------------+
| Topic                 | Topic                 | Yes             |
+-----------------------+-----------------------+-----------------+
| Topic                 | Group                 | No              |
+-----------------------+-----------------------+-----------------+
| Group                 | Instance              | Yes             |
+-----------------------+-----------------------+-----------------+
| Group                 | Topic                 | Yes             |
+-----------------------+-----------------------+-----------------+
| Group                 | Group                 | Yes             |
+-----------------------+-----------------------+-----------------+

Additionally, the coherent_access and ordered_access of the Subscriber can only be enabled if they are also enabled on
the Publisher.

.. _readerdatalifecycleqospolicy:

ReaderDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataReader with respect to the lifecycle of the data instances it manages
, that is, the instance that have been received and for which the DataReader maintains some internal resources.
The DataReader maintains the samples that have not been taken by the application, subject to the constraints imposed by
:ref:`historyqospolicy` and :ref:`resourcelimitsqospolicy`.

Under normal circumstances, the DataReader can only reclaim the resources associated with data instances if there are
no writers and all the samples have been taken. But this fact can cause problems if the application doesn't take those
samples as the service will prevent the DataReader from reclaiming the resources and they will remain in the DataReader
indefinitely. This QoS exist to avoid that situation.

List of QoS Policy data members:

+-----------------------------------+--------------------------------+---------------------+
| Data Member Name                  | Type                           | Default Value       |
+===================================+================================+=====================+
| autopurge_no_writer_samples_delay | fastrtps::Duration_t           | c_TimeInfinite      |
+-----------------------------------+--------------------------------+---------------------+
| autopurge_disposed_samples_delay  | fastrtps::Duration_t           | c_TimeInfinite      |
+-----------------------------------+--------------------------------+---------------------+

* **Autopurge no writers samples delay**: Defines the maximum duration the DataReader must retain the information
  regarding an instance once its `instance_state` becomes `NOT_ALIVE_NO_WRITERS`.
  After this time elapses, the DataReader purges all the internal information of the instance, including the untaken
  samples that will be lost.
* **Autopurge disposed samples delay**: Defines the maximum duration the DataReader must retain the information
  regarding an instance once its `instance_state` becomes `NOT_ALIVE_DISPOSED`.
  After this time elapses, the DataReader purges all the samples for the instance.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entity.


.. _reliabilityqospolicy:

ReliabilityQosPolicy
^^^^^^^^^^^^^^^^^^^^

This QoS Policy indicates the level of reliability offered and requested by the service.

List of QoS Policy data members:

+-----------------------------------+--------------------------------+----------------------------------+
| Data Member Name                  | Type                           | Default Value                    |
+===================================+================================+==================================+
|                                   |                                | BEST_EFFORT for DataReaders      |
| kind                              | :ref:`reliabilityqospolicykind`| :raw-html:`<br />`               |
|                                   |                                | RELIABLE for DataWriters         |
+-----------------------------------+--------------------------------+----------------------------------+
| max_blocking_time                 | fastrtps::Duration_t           | 100 ms                           |
+-----------------------------------+--------------------------------+----------------------------------+

* **Kind**: Specifies the behavior of the service regarding delivery of the samples.
  See :ref:`reliabilityqospolicykind` for further details.
* **Max blocking time**: Configures the maximum duration that the write operation can be blocked.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`reliability_compatibilityrule`
    for further details.

.. _reliabilityqospolicykind:

ReliabilityQosPolicyKind
""""""""""""""""""""""""

There are two possible values:

* **Best Effort**: It indicates that it is acceptable not to retransmit the missing samples, so the messages are sent
  without waiting for an arrival confirmation.
  Presumably new values for the samples are generated often enough that it is not necessary to re-send any sample.
  However, the data samples sent by the same DataWriter will be stored in the DataReader history in the same order they
  occur.
  In other words, even if the DataReader miss some data samples, a newer value will never be change to an older value.
* **Reliable**: It indicates that the service will attempt to deliver all samples of the DataWriter's history expecting
  an arrival confirmation from the DataReader.
  The data samples sent by the same DataWriter cannot be made available to the DataReader if there are previous samples
  that have not been received yet.
  The service will retransmit the lost data samples in order to reconstruct a correct snapshot of the DataWriter
  history before it is accessible by the DataReader.

  This option may block the write operation, hence the `max_blocking_time` is set that will unblock it once the time
  expires.

.. _reliability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between ReliabilityQosPolicy in DataReaders and DataWriters, the DataWriter kind
must be higher or equal to the DataReader kind.
And the order between the different kinds is::

 BEST_EFFORT < RELIABLE

Table with the possible combinations:

+-----------------------+-----------------------+-----------------+
| DataWriter kind       | DataReader kind       | Compatibility   |
+=======================+=======================+=================+
| Best Effort           | Best Effort           | Yes             |
+-----------------------+-----------------------+-----------------+
| Best Effort           | Reliable              | No              |
+-----------------------+-----------------------+-----------------+
| Reliable              | Best Effort           | Yes             |
+-----------------------+-----------------------+-----------------+
| Reliable              | Reliable              | Yes             |
+-----------------------+-----------------------+-----------------+

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RELIABILITY_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RELIABILITY
    :end-before: <!--><-->

.. _resourcelimitsqospolicy:

ResourceLimitsQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy controls the resources that the service can use in order to meet the requirements imposed by the
application and other QoS Policies.

List of QoS Policy data members:

+--------------------------+-----------------------------+-----------------------+
| Data Member Name         | Type                        | Default Value         |
+==========================+=============================+=======================+
| max_samples              | int32_t                     | 5000                  |
+--------------------------+-----------------------------+-----------------------+
| max_instances            | int32_t                     | 10                    |
+--------------------------+-----------------------------+-----------------------+
| max_samples_per_instance | int32_t                     | 400                   |
+--------------------------+-----------------------------+-----------------------+
| allocated_samples        | int32_t                     | 100                   |
+--------------------------+-----------------------------+-----------------------+

* **Max samples**: Controls the maximum number of samples that the DataWriter or DataReader can manage across all the
  instances associated with it.
  In other words, it represent the maximum samples that the middleware can store for a DataReader or DataWriter.
* **Max instances**: Controls the maximum number of instances that a DataWriter or DataReader can manage.
* **Max samples per instance**: Controls the maximum number of samples within an instance that the DataWriter or
  DataReader can manage.
* **Allocated samples**: States the number of samples currently allocated.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _resourcelimits_consistencyrule:

Consistency Rule
""""""""""""""""

To maintain the consistency within the ResourceLimitsQosPolicy, the values of the data members must follow the next
conditions:

* The value of `max_samples` must be higher or equal to the value of `max_samples_per_instance`.
* The value established for the :ref:`historyqospolicy` `depth` must be lower or equal to the value stated for
  `max_samples_per_instance`.


Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RESOURCE_LIMITS_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RESOURCE_LIMITS
    :end-before: <!--><-->

.. _timebasedfilterqospolicy:

TimeBasedFilterQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

Filter that allows a DataReader to specify that it is interested only in a subset of the values of the data.
This filter states that the DataReader doesn't want to receive more than one value each `minimum_separation`,
regardless of how fast the changes occur.

The `minimum_separation` must be lower than the :ref:`deadlineqospolicy` `period`.
By default, the `minimum_separation` is zero, which means that the DataReader is potentially interested in all the
values.

List of QoS Policy data members:

+--------------------------+------------------------------+-------------------+
| Data Member Name         | Type                         | Default Value     |
+==========================+==============================+===================+
| minimum_separation       | fastrtps::Duration_t         | c_TimeZero        |
+--------------------------+------------------------------+-------------------+

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entity.

.. _topicdataqospolicy:

TopicDataQosPolicy
^^^^^^^^^^^^^^^^^^

Allow the application to attach additional information to a created Topic so that when a remote application discovers
it can access the data and use it.

List of QoS Policy data members:

+--------------------------+------------------------------+-------------------+
| Data Member Name         | Type                         | Default Value     |
+==========================+==============================+===================+
| collection               | std::vector<fastrtps::octet> | Empty vector      |
+--------------------------+------------------------------+-------------------+

.. note::
    This QoS Policy concerns to Topic entities.
    :raw-html:`<br />`
    It can be changed even if it is already created.


Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TOPIC_DATA_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _transportpriorityqospolicy:

TransportPriorityQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

The purpose of this QoS Policy is to allow the service to take advantage of those transports capable of sending
messages with different priorities. It establishes the priority of the underlying transport used to send the data.

You can choose any value within the 32-bit range for the priority. The higher the value, the higher the priority.

List of QoS Policy data members:

+--------------------------+------------------------------+-------------------+
| Data Member Na           | Type                         | Default Value     |
+==========================+==============================+===================+
| value                    | uint32_t                     | 0                 |
+--------------------------+------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Topic and DataWriter entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entity.


.. _userdataqospolicy:

UserDataQosPolicy
^^^^^^^^^^^^^^^^^

Allow the application to attach additional information to the Entity object so that when the entity is discovered
the remote application can access the data and use it.
For example, it can be used to attach the security credentials to authenticate the source from the remote application.

List of QoS Policy data members:

+--------------------------+------------------------------+-------------------+
| Data Member Name         | Type                         | Default Value     |
+==========================+==============================+===================+
| collection               | std::vector<fastrtps::octet> | Empty vector      |
+--------------------------+------------------------------+-------------------+

.. note::
    This QoS Policy concerns to all DDS entities.
    :raw-html:`<br />`
    It can be changed even if they are already created.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_USER_DATA_QOS_POLICY
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML by the moment.

.. _writerdatalifecycleqospolicy:

WriterDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataWriter with respect to the lifecycle of the data instances it manages
, that is, the instance that has been either explicitly registered with the DataWriter using the register operations
or implicitly by directly writing data.

The `autodispose_unregistered_instances` controls whether a DataWriter will automatically dispose an instance each time
it is unregistered. Even if it is disabled, the application can get the same result if uses the dispose operation
before unregistering the instance.

List of QoS Policy data members:

+------------------------------------+------------------------------+-------------------+
| Data Member Name                   | Type                         | Default Value     |
+====================================+==============================+===================+
| autodispose_unregistered_instances | bool                         | true              |
+------------------------------------+------------------------------+-------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It can be changed after the creation of the entity.




.. _extensions:

Extension QoS Policies
----------------------

This section explains all the QoS Policies used in Fast DDS which don't belong to the DDS Standard. The extensions
can be divided into two groups: those defined in the XTypes standard and those that allow the RTPS layer configuration
from the DDS layer.

.. _eprosima_extensions:

eProsima Extensions
^^^^^^^^^^^^^^^^^^^

The eProsima QoS Policies extensions are those that allow changing the values of the RTPS layer configurable settings.

.. _disablepositiveacksqospolicy:

DisablePositiveACKsQosPolicy
""""""""""""""""""""""""""""

This additional QoS allows reducing network traffic when strict reliable communication is not required and bandwidth is
limited.
It consists in changing the default behavior by which positive acks are sent from readers to writers.
Instead, only negative acks will be sent when a reader is missing a sample, but writers will keep data for a sufficient
time before considering it as acknowledged.

List of QoS Policy data members:

+------------------------------------+------------------------------+-------------------+
| Data Member Name                   | Type                         | Default Value     |
+====================================+==============================+===================+
| enabled                            | bool                         | false             |
+------------------------------------+------------------------------+-------------------+
| duration                           | fastrtps::Duration_t         | c_TimeInfinite    |
+------------------------------------+------------------------------+-------------------+

* **Enabled**: Specifies if the QoS is enabled or not. If it is true means that the positive acks are disabled and the
  DataReader only sends negative acks. Otherwise, both positive and negative acks are sent.
* **Duration**: State the duration that the DataWriters keep the data before considering it as acknowledged.

.. note::
     This QoS Policy concerns to DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`disableacks_compatibilityrule`
    for further details.

.. _disableacks_compatibilityrule:

Compatibility Rule
******************

To maintain the compatibility between DisablePositiveACKsQosPolicy in DataReaders and DataWriters, the DataReader
cannot have this QoS enabled if the DataWriter have it disabled.

Table with the possible combinations:

+----------------------------+----------------------------+-----------------+
| DataWriter `enabled` value | DataReader `enabled` value | Compatibility   |
+============================+============================+=================+
| true                       | true                       | Yes             |
+----------------------------+----------------------------+-----------------+
| true                       | false                      | Yes             |
+----------------------------+----------------------------+-----------------+
| false                      | true                       | No              |
+----------------------------+----------------------------+-----------------+
| false                      | false                      | Yes             |
+----------------------------+----------------------------+-----------------+

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DISABLE_POSITIVE_ACKS_QOS_POLICY
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_DISABLE_POSITIVE_ACKS
    :end-before: <!--><-->

.. _participantresourcelimitsqos:

ParticipantResourceLimitsQos
""""""""""""""""""""""""""""

This QoS configures allocation limits and the use of physical memory for internal resources.

List of QoS Policy data members:

+-------------------------+-------------------------------------------+
| Data Member Name        | Type                                      |
+=========================+===========================================+
| locators                | :ref:`remotelocatorsallocationattributes` |
+-------------------------+-------------------------------------------+
| participants            | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| readers                 | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| writers                 | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| send_buffers            | :ref:`sendbuffersallocationattributes`    |
+-------------------------+-------------------------------------------+
| data_limits             | :ref:`variablelengthdatalimits`           |
+-------------------------+-------------------------------------------+

* **Locators**: Defines the limits for collections of remote locators.
* **Participants**: Specifies the allocation behavior and limits for collections dependent on the total number of
  participants.
* **Readers**: Specifies the allocation behavior and limits for collections dependent on the total number of
  readers per participant.
* **Writers**: Specifies the allocation behavior and limits for collections dependent on the total number of
  writers per participant.
* **Send buffers**: Defines the allocation behavior and limits for the send buffer manager.
* **Data Limits**: States the limits for variable-length data.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _remotelocatorsallocationattributes:

RemoteLocatorsAllocationAttributes
**********************************

This structure holds the limits for the remote locators' collections.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| max_unicast_locators               | size_t                       | 4                 |
+------------------------------------+------------------------------+-------------------+
| max_multicast_locators             | size_t                       | 1                 |
+------------------------------------+------------------------------+-------------------+

* **Max unicast locators**: This member controls the maximum number of unicast locators to keep for each discovered
  remote entity.
  It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.
* **Max multicast locators**: This member controls the maximum number of multicast locators to keep for each discovered
  remote entity.
  The default value is usually enough, as it doesn't make sense to add more than one multicast locator per entity.


.. _resourcelimitedcontainerconfig:

ResourceLimitedContainerConfig
******************************

This structure holds the limits of a resource limited collection, as well as the allocation configuration that can be
fixed size or dynamic size.

List of structure members:

+------------------------------------+------------------------------+-----------------------------------+
| Member Name                        | Type                         | Default Value                     |
+====================================+==============================+===================================+
| initial                            | size_t                       | 0                                 |
+------------------------------------+------------------------------+-----------------------------------+
| maximum                            | size_t                       | std::numeric_limits<size_t>::max()|
+------------------------------------+------------------------------+-----------------------------------+
| increment                          | size_t                       | 1 (dynamic size), 0 (fixed size)  |
+------------------------------------+------------------------------+-----------------------------------+

* **Initial**: Indicates the number of elements to preallocate in the collection.
* **Maximum**: Specifies the maximum number of elements allowed in the collection.
* **Increment**: States the number of items to add when the capacity limit is reached. This member has a different
  default value depending on the allocation configuration chosen.

.. _sendbuffersallocationattributes:

SendBuffersAllocationAttributes
*******************************

This structure holds the limits for the allocations' send buffers.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| preallocated_number                | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| dynamic                            | bool                         | false             |
+------------------------------------+------------------------------+-------------------+

* **Preallocated number**: This member controls the initial number of send buffers to be allocated.
  The default value will perform an initial guess of the number of buffers required, based on the number of threads
  from which a send operation could be started.
* **Dynamic**: This member controls how the buffer manager behaves when a send buffer is not available.
  When true, a new buffer will be created. Otherwise, it will wait for a buffer to be returned.

.. _variablelengthdatalimits:

VariableLengthDataLimits
************************

This structure holds the limits for variable-length data.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| max_properties                     | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| max_user_data                      | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| max_partitions                     | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+

* **Max properties**: Defines the maximum size, in octets, of the properties data in the local or remote participant.
* **Max user data**: Establishes the maximum size, in octets, of the user data in the local or remote participant.
* **Max partitions**: States the maximum size, in octets, of the partitions data.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PARTICIPANT_RESOURCE_LIMITS_QOS_POLICY
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-ALLOCATION-QOS-EXAMPLE
    :end-before: <publisher

.. _propertypolicyqos:

PropertyPolicyQos
"""""""""""""""""

This additional QoS Policy stores name/value pairs that can be used to configure certain DDS settings that cannot
be configured directly using an standard QoS Policy.
In Fast DDS, it can be used to configure the security settings (See :ref:`security` for further details of the security
functionality).

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PROPERTY_POLICY_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_PROPERTY_POLICY
    :end-before: <!--><-->

.. _publishmodeqospolicy:

PublishModeQosPolicy
""""""""""""""""""""

This QoS Policy configure how the middleware sends the application data.

List of QoS Policy data members:

+--------------------------+--------------------------------+-----------------------+
| Data Member Name         | Type                           | Default Value         |
+==========================+================================+=======================+
| kind                     | :ref:`publishmodeqospolicykind`| SYNCHRONOUS           |
+--------------------------+--------------------------------+-----------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _publishmodeqospolicykind:

PublishModeQosPolicyKind
************************

There are two possible values:

* **Synchronous**: The data is sent in the context of the user thread, which calls the write operation.
* **Asynchronous**: An internal thread takes the responsibility of sending the data asynchronously.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PUBLISH_MODE_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-QOS-PUBLISHMODE<-->
    :end-before: <!--><-->

.. _readerresourcelimitsqos:

ReaderResourceLimitsQos
"""""""""""""""""""""""

This QoS Policy states the limits for the matched DataWriters' resource limited collections based on the maximum number
of DataWriters that are going to match with the DataReader.

List of QoS Policy data members:

+------------------------------+-------------------------------------------+
| Data Member Name             | Type                                      |
+==============================+===========================================+
| matched_publisher_allocation | :ref:`resourcelimitedcontainerconfig`     |
+------------------------------+-------------------------------------------+


.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_READER_RESOURCE_LIMITS_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_READER_RESOURCE_LIMITS_QOS<-->
    :end-before: <!--><-->

.. _rtpsendpointqos:

RTPSEndpointQos
"""""""""""""""

This QoS Policy configures the aspects of an RTPS endpoint, such as the list of locators, the identifiers and the
history memory policy.

List of QoS Policy data members:

+--------------------------+--------------------------------+-----------------------+
| Data Member Name         | Type                           | Default Value         |
+==========================+================================+=======================+
| unicast_locator_list     | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| multicast_locator_list   | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| remote_locator_list      | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| user_defined_id          | int16_t                        | -1                    |
+--------------------------+--------------------------------+-----------------------+
| entity_id                | int16_t                        | -1                    |
+--------------------------+--------------------------------+-----------------------+
| history_memory_policy    | :ref:`memorymanagementpolicy`  | PREALLOCATED          |
+--------------------------+--------------------------------+-----------------------+

* **Unicast locator list**: Defines the list of unicast locators associated to the DDS Entity.
  DataReaders and DataWriters inherit the list of unicast locators set in the DomainParticipant, but it can be
  changed by means of this QoS.
* **Multicast locator list**: Stores the list of multicast locators associated to the DDS Entity.
  By default, DataReaders and DataWriters don't use any multicast locator, but it can be changed by means of this QoS.
* **Remote locator list**: States the list of remote locators associated to the DDS Entity.
* **User defined ID**: Establishes the unique identifier used for StaticEndpointDiscovery.
* **Entity ID**: The user can specify the identifier for the endpoint.
* **History memory policy**: Indicates the way the memory is managed in terms of dealing with the CacheChanges.

.. note::
     This QoS Policy concerns to DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _memorymanagementpolicy:

MemoryManagementPolicy
**********************

There are four possible values:

* **Preallocated**: This option sets the size to the maximum of each data type. It produces the largest memory
  footprint but the smallest allocation count.
* **Preallocated with realloc**: This option set the size to the default for each data type and it requires
  reallocation when a bigger message arrives. It produces a lower memory footprint at the expense of increasing the
  allocation count.
* **Dynamic reserve**: This option allocates dynamically the size at the time of message arrival. It produces the least
  memory footprint but the highest allocation count.
* **Dynamic reusable**: This option is similar to the previous one, but the allocated memory is reused for future
  messages.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_ENDPOINT_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_ENDPOINT_QOS<-->
    :end-before: <!--><-->

.. _rtpsreliablereaderqos:

RTPSReliableReaderQos
"""""""""""""""""""""

This RTPS QoS Policy allows the configuration of several RTPS reliable reader's aspects.

List of QoS Policy data members:

+--------------------------+------------------------------------+
| Data Member Name         | Type                               |
+==========================+====================================+
| times                    | :ref:`readertimes`                 |
+--------------------------+------------------------------------+
| disable_positive_ACKs    | :ref:`disablepositiveacksqospolicy`|
+--------------------------+------------------------------------+

* **Times**: Defines the duration of the RTPSReader events. See :ref:`readertimes` for further details.
* **Disable positive ACKs**: Configure the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _readertimes:

ReaderTimes
***********

This structure defines the times associated with the Reliable Readers' events.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| initialAcknackDelay                | fastrtps::Duration_t         | 70 milliseconds   |
+------------------------------------+------------------------------+-------------------+
| heartbeatResponseDelay             | fastrtps::Duration_t         | 5 milliseconds    |
+------------------------------------+------------------------------+-------------------+

* **Initial acknack delay**: Defines duration of the initial acknack delay.
* **Heartbeat response delay**: Establishes the duration of the delay applied when a heartbeat message is received.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_RELIABLE_READER_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_RELIABLE_READER_QOS<-->
    :end-before: <!--><-->


.. _rtpsreliablewriterqos:

RTPSReliableWriterQos
"""""""""""""""""""""

This RTPS QoS Policy allows the configuration of several RTPS reliable writer's aspects.

List of QoS Policy data members:

+--------------------------+------------------------------------+
| Data Member Name         | Type                               |
+==========================+====================================+
| times                    | :ref:`writertimes`                 |
+--------------------------+------------------------------------+
| disable_positive_acks    | :ref:`disablepositiveacksqospolicy`|
+--------------------------+------------------------------------+

* **Times**: Defines the duration of the RTPSWriter events. See :ref:`writertimes` for further details.
* **Disable positive ACKs**: Configure the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _writertimes:

WriterTimes
***********

This structure defines the times associated with the Reliable Writers' events.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| initialHeartbeatDelay              | fastrtps::Duration_t         | 12 milliseconds   |
+------------------------------------+------------------------------+-------------------+
| heartbeatPeriod                    | fastrtps::Duration_t         | 3 seconds         |
+------------------------------------+------------------------------+-------------------+
| nackResponseDelay                  | fastrtps::Duration_t         | 5 milliseconds    |
+------------------------------------+------------------------------+-------------------+
| nackSupressionDuration             | fastrtps::Duration_t         | 0 seconds         |
+------------------------------------+------------------------------+-------------------+

* **Initial heartbeat delay**: Defines duration of the initial heartbeat delay.
* **Heartbeat period**: Specifies the interval between periodic heartbeats.
* **Nack response delay**: Establishes the duration of the delay applied to the response of an ACKNACK message.
* **Nack supression duration**: The RTPSWriter ignore the nack messages received after sending the data until the
  duration time elapses.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_RELIABLE_WRITER_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_RELIABLE_WRITER_QOS<-->
    :end-before: <!--><-->

.. _transportconfigqos:

TransportConfigQos
""""""""""""""""""

This QoS Policy allows the configuration of the transport layer settings.

List of QoS Policy data members:

+---------------------------+------------------------------------------------------------------+-----------------+
| Data Member Name          | Type                                                             | Default Value   |
+===========================+==================================================================+=================+
| user_transports           | std::vector<std::shared_ptr<:ref:`transportdescriptorinterface`>>| Empty Vector    |
+---------------------------+------------------------------------------------------------------+-----------------+
| use_builtin_transports    | bool                                                             | true            |
+---------------------------+------------------------------------------------------------------+-----------------+
| send_socket_buffer_size   | uint32_t                                                         | 0               |
+---------------------------+------------------------------------------------------------------+-----------------+
| listen_socket_buffer_size | uint32_t                                                         | 0               |
+---------------------------+------------------------------------------------------------------+-----------------+

* **User transports**: This data member defines the list of transports to use alongside or in place of builtins.
* **Use builtin transports**: It controls whether the built-in transport layer is enabled or disabled. If it is set to
  false, the default UDPv4 implementation is disabled.
* **Send socket buffer size**: By default, Fast DDS creates socket buffers using the system default size. This data
  member allows to change the send socket buffer size used to send data.
* **Listen socket buffer size**: The listen socket buffer size is also created with the system default size, but it can
  be changed using this data member.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _transportdescriptorinterface:

TransportDescriptorInterface
****************************

This structure is the base for the data type used to define transport configuration.

List of structure members:

+------------------------------------+------------------------------+
| Member Name                        | Type                         |
+====================================+==============================+
| maxMessageSize                     | uint32_t                     |
+------------------------------------+------------------------------+
| maxInitialPeersRange               | uint32_t                     |
+------------------------------------+------------------------------+

* **Max message size**: This member sets the maximum size in bytes of the transport's message buffer.
* **Max initial peers range**: This member states the maximum number of guessed initial peers to try to connect.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TRANSPORT_CONFIG_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-COMMON-TRANSPORT-SETTING<-->
    :end-before: <!--><-->

.. _typeconsistencyqos:

TypeConsistencyQos
""""""""""""""""""

This QoS Policy allow the configuration of the DataReader XTypes extension QoS.

List of QoS Policy data members:

+--------------------------------+-------------------------------------------+
| Data Member Name               | Type                                      |
+================================+===========================================+
| type_consistency               | :ref:`typeconsistencyenforcementqospolicy`|
+--------------------------------+-------------------------------------------+
| representation                 | :ref:`datarepresentationqospolicy`        |
+--------------------------------+-------------------------------------------+

* **Type consistency**: It states the rules for the data types compatibility.
  See :ref:`typeconsistencyenforcementqospolicy` for further details.
* **Representation**: It specifies the data representations valid for the entities.
  See :ref:`datarepresentationqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TYPE_CONSISTENCY_QOS
   :end-before: //!

XML
###
This QoS Policy cannot be configured using XML by the moment.

.. _wireprotocolconfigqos:

WireProtocolConfigQos
"""""""""""""""""""""

This QoS Policy allows the configuration of the wire protocol.

List of QoS Policy data members:

+--------------------------------+---------------------------------------+-----------------------+
| Data Member Name               | Type                                  | Default Value         |
+================================+=======================================+=======================+
| prefix                         | fastrtps::rtps::GuidPrefix_t          | 0                     |
+--------------------------------+---------------------------------------+-----------------------+
| participant_id                 | int32_t                               | -1                    |
+--------------------------------+---------------------------------------+-----------------------+
| builtin                        | :ref:`DS_BuiltinAttributes`           |                       |
+--------------------------------+---------------------------------------+-----------------------+
| throughput_controller          | :ref:`throughputcontrollerdescription`|                       |
+--------------------------------+---------------------------------------+-----------------------+
| default_unicast_locator_list   | fastrtps::rtps::LocatorList_t         | Empty List            |
+--------------------------------+---------------------------------------+-----------------------+
| default_multicast_locator_list | fastrtps::rtps::LocatorList_t         | Empty List            |
+--------------------------------+---------------------------------------+-----------------------+

* **Prefix**: This data member allows the user to set manually the GUID prefix.
* **Participant ID**: It sets the participant identifier. By default, it will be automatically generated by the Domain.
* **Builtin**: This data member allows the configuration of the built-in parameters.
  See :ref:`DS_BuiltinAttributes` for further details.
* **Throughput controller**: It allows the configuration of the throughput settings.
* **Default unicast locator list**: States the default list of unicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without unicast locators. This list should include at
  least one locator.
* **Default multicast locator list**: Stores the default list of multicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without multicast locators. This list is usually left
  empty.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _throughputcontrollerdescription:

ThroughputControllerDescription
*******************************

This structure allows to limit the output bandwidth.

List of structure members:

+------------------------------------+------------------------------+
| Member Name                        | Type                         |
+====================================+==============================+
| bytesPerPeriod                     | uint32_t                     |
+------------------------------------+------------------------------+
| periodMillisecs                    | uint32_t                     |
+------------------------------------+------------------------------+

* **Bytes per period**: This member states the number of bytes that this controller will allow in a given period.
* **Period in milliseconds**: It specifies the window of time in which no more than `bytesPerPeriod` bytes are allowed.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_WIRE_PROTOCOL_CONFIG_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_WIRE_PROTOCOL_CONFIG_QOS<-->
    :end-before: <!--><-->

.. _writerresourcelimitsqos:

WriterResourceLimitsQos
"""""""""""""""""""""""

This QoS Policy states the limits for the matched DataReaders' resource limited collections based on the maximum number
of DataReaders that are going to match with the DataWriter.

List of QoS Policy data members:

+-------------------------------+-------------------------------------------+
| Data Member Name              | Type                                      |
+===============================+===========================================+
| matched_subscriber_allocation | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------------+-------------------------------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_WRITER_RESOURCE_LIMITS_QOS
   :end-before: //!

XML
###
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_WRITER_RESOURCE_LIMITS_QOS<-->
    :end-before: <!--><-->


.. _xtypes_extensions:

XTypes Extensions
^^^^^^^^^^^^^^^^^^

This section explain those QoS Policy extensions defined in the `XTypes Specification <https://www.omg.org/spec/DDS-XTypes/>`_:

.. _datarepresentationqospolicy:

DataRepresentationQosPolicy
"""""""""""""""""""""""""""

This XTypes QoS Policy states which data representations will be used by the DataWriters and DataReaders.

The DataWriters offered a single data representation that will be used to communicate with the DataReaders matched.
The DataReaders can request one or more data representations and in order to have communication with the DataWriter,
the offered data representation need to be contained within the DataReader sequence.

List of QoS Policy data members:

+--------------------------------+------------------------------------------+-----------------------+
| Data Member Name               | Type                                     | Default Value         |
+================================+==========================================+=======================+
| m_value                        | std::vector<:ref:`datarepresentationid`> | Empty vector          |
+--------------------------------+------------------------------------------+-----------------------+

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _datarepresentationid:

DataRepresentationId
********************

There are three possible values:

* **XCDR data representation**: This option corresponds to the first version of the `Extended CDR Representation`
  encoding.
* **XML data representation**: This option corresponds to the `XML Data Representation`.
* **XCDR2 data representation**: This option corresponds to the second version of the `Extended CDR Representation`
  encoding.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DATA_REPRESENTATION_QOS
   :end-before: //!

XML
###
This QoS Policy cannot be configured using XML by the moment.

.. _typeconsistencyenforcementqospolicy:

TypeConsistencyEnforcementQosPolicy
"""""""""""""""""""""""""""""""""""

This XTypes QoS Policy extension defines the rules for determining whether the data type used to send the data by the
DataWriter is consistent with the one used in the DataReader.

List of QoS Policy data members:

+--------------------------------+---------------------------------------+-----------------------+
| Data Member Name               | Type                                  | Default Value         |
+================================+=======================================+=======================+
| m_kind                         | :ref:`typeconsistencykind`            | ALLOW_TYPE_COERCION   |
+--------------------------------+---------------------------------------+-----------------------+
| m_ignore_sequence_bounds       | bool                                  | true                  |
+--------------------------------+---------------------------------------+-----------------------+
| m_ignore_string_bounds         | bool                                  | true                  |
+--------------------------------+---------------------------------------+-----------------------+
| m_ignore_member_names          | bool                                  | false                 |
+--------------------------------+---------------------------------------+-----------------------+
| m_prevent_type_widening        | bool                                  | false                 |
+--------------------------------+---------------------------------------+-----------------------+
| m_force_type_validation        | bool                                  | false                 |
+--------------------------------+---------------------------------------+-----------------------+

* **Kind**: It determines whether the DataWriter type must be equal to the DataReader type or not.
  See :ref:`typeconsistencykind` for further details.
* **Ignore sequence bounds**: This data member controls whether the sequence bounds are taken into account for type
  assignability or not. If its value is true, the sequences maximum lengths are not considered, which means that a
  sequence T2 with length L2 would be assignable to a sequence T1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the sequences as assignable.
* **Ignore string bounds**: It controls whether the string bounds are considered for type assignation or not.
  If its value is true, the strings maximum lengths are not considered, which means that a string S2 with length L2
  would be assignable to a string S1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the strings as assignable.
* **Ignore member names**: This boolean controls whether the member names are taken into consideration for
  type assignability or not.
  If it is true, apart from the member ID, the member names are considered as part of assignability, which means that
  the members with the same ID have also the same name. But if the value is false, the member names are ignored.
* **Prevent type widening**: This data member controls whether the type widening is allowed or not.
  If it is false, the type widening is permitted, but if true, a wider type cannot be assignable to a narrow type.
* **Force type validation**: It controls if the service need the type information to complete the matching between a
  DataWriter and a DataReader.
  If it is enabled, it must have the Complete Type Information, otherwise it is not necessary.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed after the creation of the entity.

.. _typeconsistencykind:

TypeConsistencyKind
*******************

There are two possible values:

* **Disallow type coercion**: The DataWriter and the DataReader must support the same data type in order to
  communicate.
* **Allow type coercion**: The DataWriter and the DataReader don't need to support the same data type in order to
  communicate as long as the DataReader's type is assignable from the DataWriter's type.

Example
*******

C++
###
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TYPE_CONSISTENCY_ENFORCEMENT_QOS
   :end-before: //!

XML
###
This QoS Policy cannot be configured using XML by the moment.
