.. role:: raw-html(raw)
    :format: html


.. _standard:

Standard QoS Policies
---------------------

This section explains each of the DDS standard QoS Policies:

* :ref:`deadlineqospolicy`
* :ref:`destinationorderqospolicy`
* :ref:`durabilityqospolicy`
* :ref:`durabilityserviceqospolicy`
* :ref:`entityfactoryqospolicy`
* :ref:`groupqospolicy`
* :ref:`historyqospolicy`
* :ref:`latencybudgetqospolicy`
* :ref:`lifespanqospolicy`
* :ref:`livelinessqospolicy`
* :ref:`ownershipqospolicy`
* :ref:`ownershipstrengthqospolicy`
* :ref:`partitionqospolicy`
* :ref:`presentationqospolicy`
* :ref:`readerdatalifecycleqospolicy`
* :ref:`reliabilityqospolicy`
* :ref:`resourcelimitsqospolicy`
* :ref:`timebasedfilterqospolicy`
* :ref:`topicdataqospolicy`
* :ref:`transportpriorityqospolicy`
* :ref:`userdataqospolicy`
* :ref:`writerdatalifecycleqospolicy`


.. _deadlineqospolicy:

DeadlineQosPolicy
^^^^^^^^^^^^^^^^^

This QoS policy raises an alarm when the frequency of new samples falls below a certain threshold.
It is useful for cases where data is expected to be updated periodically.

On the publishing side, the deadline defines the maximum period in which the application is expected to supply a new
sample.
On the subscribing side, it defines the maximum period in which new samples should be received.

For topics with keys, this QoS is applied by key. Imagine for example we are publishing vehicle positions, and we want
to enforce a position of each vehicle is published periodically.
In that case, we can set the ID of the vehicle as the key of the data type, and set the deadline QoS to the desired
publication period.

List of QoS Policy data members:

+--------------------------+--------------------------------------------------+-----------------------------------+
| Data Member Name         | Type                                             | Default Value                     |
+==========================+==================================================+===================================+
| period                   | fastrtps::Duration_t                             | c_TimeInfinite                    |
+--------------------------+--------------------------------------------------+-----------------------------------+

.. note::

   This QoS Policy concerns to Topic, DataReader and DataWriter entities.
   :raw-html:`<br />`
   It can be changed on enabled entities.

.. warning::

    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`deadline_compatibilityrule` for further details.

.. _deadline_compatibilityrule:

Compatibility Rule
""""""""""""""""""
To maintain the compatibility between DeadlineQosPolicy in DataReaders and DataWriters, the offered deadline period
(configured on the DataWriter) must be less than or equal to the requested deadline period (configured on the
DataReader), otherwise, the entities are considered to be incompatible.

The DeadlineQosPolicy must be set consistently with the :ref:`timebasedfilterqospolicy`, which means that the deadline
period must be higher or equal to the minimum separation.

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
    It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`destinationorder_compatibilityrule` for
    further details.

.. _destinationorderqospolicykind:

DestinationOrderQosPolicyKind
"""""""""""""""""""""""""""""

There are two possible values:

* ``BY_RECEPTION_TIMESTAMP``: This indicates that the data is ordered based on the reception time at each DataReader,
  which means that the last received value should be the one kept.
  This option may cause that each DataReader ends up with a different final value, since the DataReaders may receive
  the data at different times.
* ``BY_SOURCE_TIMESTAMP``: This indicates that the data is ordered based on the DataWriter timestamp at the time the
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
| BY_RECEPTION_TIMESTAMP | BY_RECEPTION_TIMESTAMP  | Yes             |
+------------------------+-------------------------+-----------------+
| BY_RECEPTION_TIMESTAMP | BY_SOURCE_TIMESTAMP     | No              |
+------------------------+-------------------------+-----------------+
| BY_SOURCE_TIMESTAMP    | BY_RECEPTION_TIMESTAMP  | Yes             |
+------------------------+-------------------------+-----------------+
| BY_SOURCE_TIMESTAMP    | BY_SOURCE_TIMESTAMP     | Yes             |
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
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`durability_compatibilityrule` for
    further details.

.. _durabilitykind:

DurabilityQosPolicyKind
"""""""""""""""""""""""

There are four possible values:

* ``VOLATILE_DURABILITY_QOS``: Past samples are ignored and a joining DataReader receives samples generated after the
  moment it matches.
* ``TRANSIENT_LOCAL_DURABILITY_QOS``: When a new DataReader joins, its History is filled with past samples.
* ``TRANSIENT_DURABILITY_QOS``: When a new DataReader joins, its History is filled with past samples, which are stored
  on persistent storage (see :ref:`persistence`).
* ``PERSISTENT_DURABILITY_QOS`` (`Not Implemented`): All the sample are stored on a permanent storage, so that they can
  outlive a system session.

.. _durability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DurabilityQosPolicy in DataReaders and DataWriters when they have different kind
values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is::

 VOLATILE_DURABILITY_QOS < TRANSIENT_LOCAL_DURABILITY_QOS < TRANSIENT_DURABILITY_QOS < PERSISTENT_DURABILITY_QOS

Table with the possible combinations:

+--------------------------------+--------------------------------+-----------------+
| DataWriter kind                | DataReader kind                | Compatibility   |
+================================+================================+=================+
| VOLATILE_DURABILITY_QOS        | VOLATILE_DURABILITY_QOS        | Yes             |
+--------------------------------+--------------------------------+-----------------+
| VOLATILE_DURABILITY_QOS        | TRANSIENT_LOCAL_DURABILITY_QOS | No              |
+--------------------------------+--------------------------------+-----------------+
| VOLATILE_DURABILITY_QOS        | TRANSIENT_DURABILITY_QOS       | No              |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_LOCAL_DURABILITY_QOS | VOLATILE_DURABILITY_QOS        | Yes             |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_LOCAL_DURABILITY_QOS | TRANSIENT_LOCAL_DURABILITY_QOS | Yes             |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_LOCAL_DURABILITY_QOS | TRANSIENT_DURABILITY_QOS       | No              |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_DURABILITY_QOS       | VOLATILE_DURABILITY_QOS        | Yes             |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_DURABILITY_QOS       | TRANSIENT_LOCAL_DURABILITY_QOS | Yes             |
+--------------------------------+--------------------------------+-----------------+
| TRANSIENT_DURABILITY_QOS       | TRANSIENT_DURABILITY_QOS       | Yes             |
+--------------------------------+--------------------------------+-----------------+

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
  information is kept until all the following conditions are met:

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
     It cannot be changed on enabled entities.

.. _entityfactoryqospolicy:

EntityFactoryQosPolicy
^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy controls the behavior of an :ref:`dds_layer_core_entity` when it acts as a factory for other entities.
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
     It can be changed on enabled entities, but it only affects those entities created after the change.

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
This QoS Policy cannot be configured using XML for the moment.

.. _groupqospolicy:

GroupDataQosPolicy
^^^^^^^^^^^^^^^^^^

Allows the application to attach additional information to created Publishers or Subscribers.
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
     It can be changed on enabled entities.

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
This QoS Policy cannot be configured using XML for the moment.

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
  It only has effect if the kind is set to KEEP_LAST and it needs to be consistent with the
  :ref:`resourcelimitsqospolicy`, which means that its value must be lower or equal to max_samples_per_instance.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _historyqospolicykind:

HistoryQosPolicyKind
""""""""""""""""""""
There are two possible values:

* ``KEEP_LAST_HISTORY_QOS``: The service will only attempt to keep the most recent values of the instance and discard
  the older ones.
  The maximum number of samples to keep and deliver is defined by the `depth` of the HistoryQosPolicy, which needs to
  be consistent with the :ref:`resourcelimitsqospolicy` settings.
  If the limit defined by `depth` is reached, the system will discard the oldest sample to make room for a new one.
* ``KEEP_ALL_HISTORY_QOS``: The service will attempt to keep all the values of the instance until it can be delivered
  to all the existing Subscribers.
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

This QoS Policy specifies the maximum acceptable delay from the time the data is
written until the data is inserted on the DataReader History and notified of the fact.
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
     It can be changed on enabled entities.

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
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

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
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`liveliness_compatibilityrule`
    for further details.

.. _livelinessqospolicykind:

LivelinessQosPolicyKind
"""""""""""""""""""""""

There are three possible values:

* ``AUTOMATIC_LIVELINESS_QOS``: The service takes the responsibility for renewing the leases at the required rates, as
  long as the local process where the participant is running and the link connecting it to remote participants exists,
  the entities within the remote participant will be considered alive.
  This kind is suitable for applications that only need to detect whether a remote application is still running.
* The two `Manual` modes require that the application on the publishing side asserts the liveliness periodically
  before the lease_duration timer expires.
  Publishing any new data value implicitly asserts the DataWriter's liveliness, but it can be done explicitly
  by calling the `assert_liveliness` member function.

  * ``MANUAL_BY_PARTICIPANT_LIVELINESS_QOS``: If one of the entities in the publishing side asserts its liveliness, the
    service deduces that all other entities within the same DomainParticipant are also alive.
  * ``MANUAL_BY_TOPIC_LIVELINESS_QOS``: This mode is more restrictive and requires that at least one instance within
    the DataWriter is asserted to consider that the DataWriter is alive.

.. _liveliness_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between LivelinessQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
higher or equal to the DataReader kind.
And the order between the different kinds is::

 AUTOMATIC_LIVELINESS_QOS < MANUAL_BY_PARTICIPANT_LIVELINESS_QOS < MANUAL_BY_TOPIC_LIVELINESS_QOS

Table with the possible combinations:

+--------------------------------------+--------------------------------------+-----------------+
| DataWriter kind                      | DataReader kind                      | Compatibility   |
+======================================+======================================+=================+
| AUTOMATIC_LIVELINESS_QOS             | AUTOMATIC_LIVELINESS_QOS             | Yes             |
+--------------------------------------+--------------------------------------+-----------------+
| AUTOMATIC_LIVELINESS_QOS             | MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | No              |
+--------------------------------------+--------------------------------------+-----------------+
| AUTOMATIC_LIVELINESS_QOS             | MANUAL_BY_TOPIC_LIVELINESS_QOS       | No              |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | AUTOMATIC_LIVELINESS_QOS             | Yes             |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | Yes             |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | MANUAL_BY_TOPIC_LIVELINESS_QOS       | No              |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_TOPIC_LIVELINESS_QOS       | AUTOMATIC_LIVELINESS_QOS             | Yes             |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_TOPIC_LIVELINESS_QOS       | MANUAL_BY_PARTICIPANT_LIVELINESS_QOS | Yes             |
+--------------------------------------+--------------------------------------+-----------------+
| MANUAL_BY_TOPIC_LIVELINESS_QOS       | MANUAL_BY_TOPIC_LIVELINESS_QOS       | Yes             |
+--------------------------------------+--------------------------------------+-----------------+

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
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`ownership_compatibilityrule`
    for further details.

.. _ownershipqospolicykind:

OwnershipQosPolicyKind
""""""""""""""""""""""

There are two possible values:

* ``SHARED_OWNERSHIP_QOS``: This option indicates that the service doesn't enforce unique ownership for each instance.
  In this case, multiple DataWriters are allowed to update the same data instance and all the updates are made
  available to the existing DataReaders.
  Those updates are also subject to the :ref:`timebasedfilterqospolicy` or :ref:`historyqospolicy` settings, so they
  can be filtered.
* ``EXCLUSIVE_OWNERSHIP_QOS``: This option indicates that each instance can only be updated by one DataWriter, meaning
  that at any point in time a single DataWriter owns each instance and is the only one whose modifications will be
  visible for the existing DataReaders.
  The owner can be changed dynamically according to the highest `strength` between the alive DataWriters, which has not
  violated the deadline contract concerning the data instances.
  That `strength` can be changed using the :ref:`ownershipstrengthqospolicy`.

.. _ownership_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between OwnershipQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
equal to the DataReader kind.

Table with the possible combinations:

+-------------------------+-------------------------+-----------------+
| DataWriter kind         | DataReader kind         | Compatibility   |
+=========================+=========================+=================+
| SHARED_OWNERSHIP_QOS    | SHARED_OWNERSHIP_QOS    | Yes             |
+-------------------------+-------------------------+-----------------+
| SHARED_OWNERSHIP_QOS    | EXCLUSIVE_OWNERSHIP_QOS | No              |
+-------------------------+-------------------------+-----------------+
| EXCLUSIVE_OWNERSHIP_QOS | SHARED_OWNERSHIP_QOS    | No              |
+-------------------------+-------------------------+-----------------+
| EXCLUSIVE_OWNERSHIP_QOS | EXCLUSIVE_OWNERSHIP_QOS | Yes             |
+-------------------------+-------------------------+-----------------+

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
This QoS Policy cannot be configured using XML for the moment.

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
     It can be changed on enabled entities.

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
This QoS Policy cannot be configured using XML for the moment.

.. _partitionqospolicy:

PartitionQosPolicy
^^^^^^^^^^^^^^^^^^

This Qos Policy allows the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match, but also they have to share
at least one logical partition.

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
     It can be changed on enabled entities.

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
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`presentation_compatibilityrule`
    for further details.

.. _presentationqospolicyaccessscopekind:

PresentationQosPolicyAccessScopeKind
""""""""""""""""""""""""""""""""""""

There are three possible values, which have different behaviors depending on the values of coherent_access and
ordered_access variables:

* ``INSTANCE_PRESENTATION_QOS``: The changes to a data instance do not need to be coherent nor ordered with respect to
  the changes to any other instance, which means that the order and coherent changes apply to each instance separately.

  * Enabling the `coherent_access`, in this case, has no effect on how the subscriber can access the data as the scope
    is limited to each instance, changes to separate instances are considered independent and thus cannot be grouped
    by a coherent change.
  * Enabling the `ordered_access`, in this case, only affects to the changes within the same instance.
    Therefore, the changes made to two instances are not necessarily seen in the order they occur even if the same
    application thread and DataWriter made them.

* ``TOPIC_PRESENTATION_QOS``: The scope spans to all the instances within the same DataWriter.

  * Enabling the `coherent_access` makes that the grouping made with changes within the same DataWriter will be
    available as coherent with respect to other changes to instances in that DataWriter, but will not be grouped with
    changes made to instances belonging to different DataWriters.
  * Enabling the `ordered_access` means that the changes made by a single DataWriter are made available to the
    subscribers in the same order that they occur, but the changes made to instances through different DataWriters are
    not necessarily seen in order.

* ``GROUP_PRESENTATION_QOS``: The scope spans to all the instances belonging to DataWriters within the same Publisher.

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

  INSTANCE_PRESENTATION_QOS < TOPIC_PRESENTATION_QOS < GROUP_PRESENTATION_QOS

Table with the possible combinations:

+----------------------------+----------------------------+-----------------+
| Publisher scope            | Subscriber scope           | Compatibility   |
+============================+============================+=================+
| INSTANCE_PRESENTATION_QOS  | INSTANCE_PRESENTATION_QOS  | Yes             |
+----------------------------+----------------------------+-----------------+
| INSTANCE_PRESENTATION_QOS  | TOPIC_PRESENTATION_QOS     | No              |
+----------------------------+----------------------------+-----------------+
| INSTANCE_PRESENTATION_QOS  | GROUP_PRESENTATION_QOS     | No              |
+----------------------------+----------------------------+-----------------+
| TOPIC_PRESENTATION_QOS     | INSTANCE_PRESENTATION_QOS  | Yes             |
+----------------------------+----------------------------+-----------------+
| TOPIC_PRESENTATION_QOS     | TOPIC_PRESENTATION_QOS     | Yes             |
+----------------------------+----------------------------+-----------------+
| TOPIC_PRESENTATION_QOS     | GROUP_PRESENTATION_QOS     | No              |
+----------------------------+----------------------------+-----------------+
| GROUP_PRESENTATION_QOS     |  INSTANCE_PRESENTATION_QOS | Yes             |
+----------------------------+----------------------------+-----------------+
| GROUP_PRESENTATION_QOS     | TOPIC_PRESENTATION_QOS     | Yes             |
+----------------------------+----------------------------+-----------------+
| GROUP_PRESENTATION_QOS     | GROUP_PRESENTATION_QOS     | Yes             |
+----------------------------+----------------------------+-----------------+

Additionally, the coherent_access and ordered_access of the Subscriber can only be enabled if they are also enabled on
the Publisher.

.. _readerdatalifecycleqospolicy:

ReaderDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataReader with respect to the lifecycle of the data instances it manages
, that is, the instances that have been received and for which the DataReader maintains some internal resources.
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
     It can be changed on enabled entities.


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
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`reliability_compatibilityrule`
    for further details.

.. _reliabilityqospolicykind:

ReliabilityQosPolicyKind
""""""""""""""""""""""""

There are two possible values:

* ``BEST_EFFORT_RELIABILITY_QOS``: It indicates that it is acceptable not to retransmit the missing samples, so the
  messages are sent without waiting for an arrival confirmation.
  Presumably new values for the samples are generated often enough that it is not necessary to re-send any sample.
  However, the data samples sent by the same DataWriter will be stored in the DataReader history in the same order they
  occur.
  In other words, even if the DataReader misses some data samples, an older value will never overwrite a newer value.
* ``RELIABLE_RELIABILITY_QOS``: It indicates that the service will attempt to deliver all samples of the DataWriter's
  history expecting an arrival confirmation from the DataReader.
  The data samples sent by the same DataWriter cannot be made available to the DataReader if there are previous samples
  that have not been received yet.
  The service will retransmit the lost data samples in order to reconstruct a correct snapshot of the DataWriter
  history before it is accessible by the DataReader.

  This option may block the write operation, hence the `max_blocking_time` is set that will unblock it once the time
  expires.
  But if the `max_blocking_time` expires before the data is sent, the write operation will return an error.

.. _reliability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between ReliabilityQosPolicy in DataReaders and DataWriters, the DataWriter kind
must be higher or equal to the DataReader kind.
And the order between the different kinds is::

 BEST_EFFORT_RELIABILITY_QOS < RELIABLE_RELIABILITY_QOS

Table with the possible combinations:

+-----------------------------+-----------------------------+-----------------+
| DataWriter kind             | DataReader kind             | Compatibility   |
+=============================+=============================+=================+
| BEST_EFFORT_RELIABILITY_QOS | BEST_EFFORT_RELIABILITY_QOS | Yes             |
+-----------------------------+-----------------------------+-----------------+
| BEST_EFFORT_RELIABILITY_QOS | RELIABLE_RELIABILITY_QOS    | No              |
+-----------------------------+-----------------------------+-----------------+
| RELIABLE_RELIABILITY_QOS    | BEST_EFFORT_RELIABILITY_QOS | Yes             |
+-----------------------------+-----------------------------+-----------------+
| RELIABLE_RELIABILITY_QOS    | RELIABLE_RELIABILITY_QOS    | Yes             |
+-----------------------------+-----------------------------+-----------------+

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
  In other words, it represents the maximum samples that the middleware can store for a DataReader or DataWriter.
* **Max instances**: Controls the maximum number of instances that a DataWriter or DataReader can manage.
* **Max samples per instance**: Controls the maximum number of samples within an instance that the DataWriter or
  DataReader can manage.
* **Allocated samples**: States the number of samples that will be allocated on initialization.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

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
     It can be changed on enabled entities.

.. _topicdataqospolicy:

TopicDataQosPolicy
^^^^^^^^^^^^^^^^^^

Allows the application to attach additional information to a created Topic so that when it is discovered by a remote
application, it can access the data and use it.

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
This QoS Policy cannot be configured using XML for the moment.

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
| Data Member Name         | Type                         | Default Value     |
+==========================+==============================+===================+
| value                    | uint32_t                     | 0                 |
+--------------------------+------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Topic and DataWriter entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.


.. _userdataqospolicy:

UserDataQosPolicy
^^^^^^^^^^^^^^^^^

Allows the application to attach additional information to the Entity object so that when the entity is discovered
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
    It can be changed on enabled entities.

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
This QoS Policy cannot be configured using XML for the moment.

.. _writerdatalifecycleqospolicy:

WriterDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataWriter with respect to the lifecycle of the data instances it manages
, that is, the instance that has been either explicitly registered with the DataWriter using the register operations
or implicitly by directly writing data.

The `autodispose_unregistered_instances` controls whether a DataWriter will automatically dispose an instance each time
it is unregistered. Even if it is disabled, the application can still get the same result if it uses the dispose
operation before unregistering the instance.

List of QoS Policy data members:

+------------------------------------+------------------------------+-------------------+
| Data Member Name                   | Type                         | Default Value     |
+====================================+==============================+===================+
| autodispose_unregistered_instances | bool                         | true              |
+------------------------------------+------------------------------+-------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

