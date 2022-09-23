.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. role:: raw-html(raw)
    :format: html


.. _standard:

Standard QoS Policies
---------------------

This section explains each of the DDS standard QoS Policies:

.. contents::
    :local:
    :backlinks: none
    :depth: 1


.. _deadlineqospolicy:

DeadlineQosPolicy
^^^^^^^^^^^^^^^^^

This QoS policy raises an alarm when the frequency of new samples falls below a certain threshold.
It is useful for cases where data is expected to be updated periodically (see |DeadlineQosPolicy-api|).

On the publishing side, the deadline defines the maximum period in which the application is expected to supply a new
sample.
On the subscribing side, it defines the maximum period in which new samples should be received.

For |Topics| with keys, this QoS is applied by key.
Suppose that the positions of some vehicles have to be published periodically.
In that case, it is possible to set the ID of the vehicle as the key of the data type and the deadline QoS to
the desired publication period.

List of QoS Policy data members:

+---------------------------------------+------------------------------------------+-----------------------------------+
| Data Member Name                      | Type                                     | Default Value                     |
+=======================================+==========================================+===================================+
| |DeadlineQosPolicy::period-api|       | |Duration_t-api|                         | |c_TimeInfinite-api|              |
+---------------------------------------+------------------------------------------+-----------------------------------+

.. note::

   This QoS Policy concerns to |Topic|, |DataReader| and |DataWriter| entities.
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

Multiple |DataWriters| can send messages in the same |Topic| using the same key, and on the |DataReader| side all those
messages are stored within the same instance of data (see |DestinationOrderQosPolicy-api|).
This QoS policy controls the criteria used to determine the logical order of those messages.
The behavior of the system depends on the value of the :ref:`destinationorderqospolicykind`.

List of QoS Policy data members:

+----------------------------------------------------+--------------------------------------+--------------------------+
| Data Member Name                                   | Type                                 | Default Value            |
+====================================================+======================================+==========================+
| |DestinationOrderQosPolicy::kind-api|              | :ref:`destinationorderqospolicykind` | |BY_RECEPTION_TIMESTAMP| |
+----------------------------------------------------+--------------------------------------+--------------------------+

.. note::
    This QoS Policy concerns to Topic, DataReader and DataWriter entities.
    :raw-html:`<br />`
    It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`destinationorder_compatibilityrule` for further details.

.. _destinationorderqospolicykind:

DestinationOrderQosPolicyKind
"""""""""""""""""""""""""""""

There are two possible values (see |DestinationOrderQosPolicyKind-api|):

* |BY_RECEPTION_TIMESTAMP|: This indicates that the data is ordered based on the reception
  time at each DataReader,
  which means that the last received value should be the one kept.
  This option may cause that each DataReader ends up with a different final value, since the DataReaders may receive
  the data at different times.
* |BY_SOURCE_TIMESTAMP|: This indicates that the data is ordered based on the DataWriter
  timestamp at the time the message is sent.
  This option guarantees the consistency of the final value.

Both options depend on the values of the :ref:`ownershipqospolicy` and :ref:`ownershipstrengthqospolicy`, meaning that
if the Ownership is set to EXCLUSIVE and the last value came from a DataWriter with low ownership strength, it will be
discarded.

.. _destinationorder_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DestinationOrderQosPolicy in DataReaders and DataWriters when they have different
kind values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is:

|BY_RECEPTION_TIMESTAMP| < |BY_SOURCE_TIMESTAMP|

Table with the possible combinations:

+---------------------------------------------------+--------------------------------------------------+---------------+
| DataWriter kind                                   | DataReader kind                                  | Compatibility |
+===================================================+==================================================+===============+
| |BY_RECEPTION_TIMESTAMP|                          | |BY_RECEPTION_TIMESTAMP|                         | Yes           |
+---------------------------------------------------+--------------------------------------------------+---------------+
| |BY_RECEPTION_TIMESTAMP|                          | |BY_SOURCE_TIMESTAMP|                            | No            |
+---------------------------------------------------+--------------------------------------------------+---------------+
| |BY_SOURCE_TIMESTAMP|                             | |BY_RECEPTION_TIMESTAMP|                         | Yes           |
+---------------------------------------------------+--------------------------------------------------+---------------+
| |BY_SOURCE_TIMESTAMP|                             | |BY_SOURCE_TIMESTAMP|                            | Yes           |
+---------------------------------------------------+--------------------------------------------------+---------------+

.. _durabilityqospolicy:

DurabilityQosPolicy
^^^^^^^^^^^^^^^^^^^

A |DataWriter| can send messages throughout a |Topic| even if there are no |DataReaders| on the network.
Moreover, a DataReader that joins to the Topic after some data has been written could be interested in accessing
that information (see |DurabilityQosPolicy-api|).

The DurabilityQoSPolicy defines how the system will behave regarding those samples that existed on the Topic before
the DataReader joins.
The behavior of the system depends on the value of the :ref:`DurabilityQosPolicyKind<durabilitykind>`.

List of QoS Policy data members:

+---------------------------------------+-----------------------+------------------------------------------------------+
| Data Member Name                      | Type                  | Default Value                                        |
+=======================================+=======================+======================================================+
|                                       |                       | |VOLATILE_DURABILITY_QOS-api| for DataReaders        |
| |DurabilityQosPolicy::kind-api|       | :ref:`durabilitykind` | :raw-html:`<br />`                                   |
|                                       |                       | |TRANSIENT_LOCAL_DURABILITY_QOS-api| for DataWriters |
+---------------------------------------+-----------------------+------------------------------------------------------+

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. important::
    In order to receive past samples in the DataReader, besides setting this Qos Policy, it is required that the
    :ref:`reliabilityqospolicy` is set to |RELIABLE_RELIABILITY_QOS-api|.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`durability_compatibilityrule` for further details.

.. _durabilitykind:

DurabilityQosPolicyKind
"""""""""""""""""""""""

There are four possible values (see |DurabilityQosPolicyKind-api|):

* |VOLATILE_DURABILITY_QOS-api|: Past samples are ignored and a joining DataReader receives samples generated after the
  moment it matches.
* |TRANSIENT_LOCAL_DURABILITY_QOS-api|: When a new DataReader joins, its History is filled with past samples.
* |TRANSIENT_DURABILITY_QOS-api|: When a new DataReader joins, its History is filled with past samples, which are stored
  on persistent storage (see :ref:`persistence_service`).
* |PERSISTENT_DURABILITY_QOS-api|: (`Not Implemented`): All the samples are stored on a permanent storage, so that they
  can outlive a system session.

.. _durability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DurabilityQosPolicy in DataReaders and DataWriters when they have different kind
values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is:

|VOLATILE_DURABILITY_QOS-api| < |TRANSIENT_LOCAL_DURABILITY_QOS-api| < |TRANSIENT_DURABILITY_QOS-api| <
|PERSISTENT_DURABILITY_QOS-api|

Table with the possible combinations:

+-------------------------------------------------+--------------------------------------------------+-----------------+
| DataWriter kind                                 | DataReader kind                                  | Compatibility   |
+=================================================+==================================================+=================+
| |VOLATILE_DURABILITY_QOS-api|                   | |VOLATILE_DURABILITY_QOS-api|                    | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |VOLATILE_DURABILITY_QOS-api|                   | |TRANSIENT_LOCAL_DURABILITY_QOS-api|             | No              |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |VOLATILE_DURABILITY_QOS-api|                   | |TRANSIENT_DURABILITY_QOS-api|                   | No              |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_LOCAL_DURABILITY_QOS-api|            | |VOLATILE_DURABILITY_QOS-api|                    | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_LOCAL_DURABILITY_QOS-api|            | |TRANSIENT_LOCAL_DURABILITY_QOS-api|             | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_LOCAL_DURABILITY_QOS-api|            | |TRANSIENT_DURABILITY_QOS-api|                   | No              |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_DURABILITY_QOS-api|                  | |VOLATILE_DURABILITY_QOS-api|                    | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_DURABILITY_QOS-api|                  | |TRANSIENT_LOCAL_DURABILITY_QOS-api|             | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+
| |TRANSIENT_DURABILITY_QOS-api|                  | |TRANSIENT_DURABILITY_QOS-api|                   | Yes             |
+-------------------------------------------------+--------------------------------------------------+-----------------+

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
|DataReader| and |DataWriter| used when the :ref:`durabilityqospolicy` kind is set to |TRANSIENT_DURABILITY_QOS-api| or
|PERSISTENT_DURABILITY_QOS-api| (see |DurabilityServiceQosPolicy-api|).

Those entities are used to simulate the persistent storage.
The fictitious DataReader reads the data written on the |Topic| and stores it, so that if the user DataWriter does not
have the information requested by the user DataReaders, the fictitious DataWriter takes care of sending that
information.

List of QoS Policy data members:

+----------------------------------------------------+-----------------------------------+-----------------------------+
| Data Member Name                                   | Type                              | Default Value               |
+====================================================+===================================+=============================+
| |service_cleanup_delay-api|                        | |Duration_t-api|                  | |c_TimeZero-api|            |
+----------------------------------------------------+-----------------------------------+-----------------------------+
| |history_kind-api|                                 | :ref:`historyqospolicykind`       | |KEEP_LAST_HISTORY_QOS-api| |
+----------------------------------------------------+-----------------------------------+-----------------------------+
| |history_depth-api|                                | ``int32_t``                       | 1                           |
+----------------------------------------------------+-----------------------------------+-----------------------------+
| |max_samples-api|                                  | ``int32_t``                       | -1 (Length Unlimited)       |
+----------------------------------------------------+-----------------------------------+-----------------------------+
| |max_instances-api|                                | ``int32_t``                       | -1 (Length Unlimited)       |
+----------------------------------------------------+-----------------------------------+-----------------------------+
| |max_samples_per_instance-api|                     | ``int32_t``                       | -1 (Length Unlimited)       |
+----------------------------------------------------+-----------------------------------+-----------------------------+

* |service_cleanup_delay-api|: It controls when the service can remove all the information regarding a data instance.
  That information is kept until all the following conditions are met:

  * The instance has been explicitly disposed and its InstanceState becomes |NOT_ALIVE_DISPOSED_INSTANCE_STATE-api|.
  * There is not any alive DataWriter writing the instance, which means that all existing writers either unregister the
    instance or lose their liveliness.
  * A time interval longer than the one established on the |service_cleanup_delay-api| has elapsed since the moment the
    service detected that the two previous conditions were met.

* |history_kind-api|: Controls the kind of the :ref:`historyqospolicy` associated with the Durability Service fictitious
  entities.
* |history_depth-api|: Controls the depth of the :ref:`historyqospolicy` associated with the Durability Service
  fictitious entities.
* |max_samples-api|: Controls the maximum number of samples of the :ref:`resourcelimitsqospolicy` associated with the
  Durability Service fictitious entities.
  This value must be higher than the maximum number of samples per instance.
* |max_instances-api|: Controls the maximum number of instances of the :ref:`resourcelimitsqospolicy` associated with
  the Durability Service fictitious entities.
* |max_samples_per_instance-api|: Controls the maximum number of samples within an instance of the
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
By default, all the entities are created enabled, but if you change the value of the |autoenable_created_entities-api|
to ``false``, the new entities will be created disabled (see |EntityFactoryQosPolicy-api|).

List of QoS Policy data members:

+---------------------------------------------------------------------------+----------+-------------------+
| Data Member Name                                                          | Type     | Default Value     |
+===========================================================================+==========+===================+
| |autoenable_created_entities-api|                                         | ``bool`` | ``true``          |
+---------------------------------------------------------------------------+----------+-------------------+

.. note::
     This QoS Policy concerns to |DomainParticipantFactory| (as factory for |DomainParticipant|), DomainParticipant
     (as factory for
     |Publisher|, |Subscriber| and |Topic|), Publisher (as factory for |DataWriter|) and Subscriber (as factory for
     |DataReader|).
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

Allows the application to attach additional information to created |Publishers| or |Subscribers|.
This data is common to all |DataWriters|/|DataReaders| belonging to the Publisher/Subscriber and it is propagated by
means of the built-in topics (see  |GroupDataQosPolicy-api|).

This QoS Policy can be used in combination with DataWriter and DataReader listeners to implement a matching policy
similar to the :ref:`PartitionQosPolicy <partitionqospolicy>`.

List of QoS Policy data members:

+--------------------------+-------------------------------+-------------------+
| Data Member Name         | Type                          | Default Value     |
+==========================+===============================+===================+
| collection               | std::vector<|octet-api|>      | Empty vector      |
+--------------------------+-------------------------------+-------------------+

.. note::
     This QoS Policy concerns to Publisher and Subscriber entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

Example
"""""""

.. tabs::

  .. tab:: C++

    .. literalinclude:: ../../../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 8
       :start-after: //DDS_CHANGE_GROUP_DATA_QOS_POLICY
       :end-before: //!

  .. tab:: XML

    .. literalinclude:: ../../../../../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_CHANGE_GROUP_DATA_QOS_POLICY
       :end-before: <!--><-->

.. _historyqospolicy:

HistoryQosPolicy
^^^^^^^^^^^^^^^^

This QoS Policy controls the behavior of the system when the value of an instance changes one or more times before it
can be successfully communicated to the existing DataReader entities.

List of QoS Policy data members:

+----------------------------------------------+-------------------------------+---------------------------------------+
| Data Member Name                             | Type                          | Default Value                         |
+==============================================+===============================+=======================================+
| |HistoryQosPolicy::kind-api|                 | :ref:`historyqospolicykind`   | |KEEP_LAST_HISTORY_QOS-api|           |
+----------------------------------------------+-------------------------------+---------------------------------------+
| |HistoryQosPolicy::depth-api|                | ``int32_t``                   | 1                                     |
+----------------------------------------------+-------------------------------+---------------------------------------+

* |HistoryQosPolicy::kind-api|: Controls if the service should deliver only the most recent values, all the
  intermediate values or do something in between.
  See :ref:`historyqospolicykind` for further details.
* |HistoryQosPolicy::depth-api|: Establishes the maximum number of samples that must be kept on the history.
  It only has effect if the kind is set to |KEEP_LAST_HISTORY_QOS-api| and it needs to be consistent with the
  :ref:`resourcelimitsqospolicy`, which means that its value must be lower or equal to max_samples_per_instance.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _historyqospolicykind:

HistoryQosPolicyKind
""""""""""""""""""""
There are two possible values (see |HistoryQosPolicyKind-api|):

* |KEEP_LAST_HISTORY_QOS-api|: The service will only attempt to keep the most recent values of the instance and discard
  the older ones.
  The maximum number of samples to keep and deliver is defined by the `depth` of the HistoryQosPolicy, which needs to
  be consistent with the :ref:`resourcelimitsqospolicy` settings.
  If the limit defined by `depth` is reached, the system will discard the oldest sample to make room for a new one.
* |KEEP_ALL_HISTORY_QOS-api|: The service will attempt to keep all the values of the instance until it can be delivered
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
That delay by default is set to 0 in order to optimize the internal operations (see |LatencyBudgetQosPolicy-api|).

List of QoS Policy data members:

+----------------------------------------------------------+---------------------------+-------------------------------+
| Data Member Name                                         | Type                      | Default Value                 |
+==========================================================+===========================+===============================+
| |LatencyBudgetQosPolicy::duration-api|                   | |Duration_t-api|          | |c_TimeZero-api|              |
+----------------------------------------------------------+---------------------------+-------------------------------+

.. note::
     This QoS Policy concerns to |Topic|, |DataWriter| and |DataReader| entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`latencybudget_compatibilityrule` for further details.

.. _latencybudget_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between LatencyBudgetQosPolicy in DataReaders and DataWriters, the DataWriter duration
must be lower or equal to the DataReader duration.

.. _lifespanqospolicy:

LifespanQosPolicy
^^^^^^^^^^^^^^^^^

Each data sample written by a |DataWriter| has an associated expiration time beyond which the data is removed from the
DataWriter and DataReader history as well as from the transient and persistent information caches
(see |LifespanQosPolicy-api|).

By default, the `duration` is infinite, which means that there is not a maximum duration for the validity of the
samples written by the DataWriter.

The expiration time is computed by adding the `duration` to the source timestamp, which can be calculated automatically
if |DataWriter::write-api| member function is called or supplied by the application by means of
:func:`write_w_timestamp` member function.
The DataReader is allowed to use the reception timestamp instead of the source timestamp.


List of QoS Policy data members:

+-----------------------------------------------------------------------+-----------------------+----------------------+
| Data Member Name                                                      | Type                  | Default Value        |
+=======================================================================+=======================+======================+
| |LifespanQosPolicy::duration-api|                                     | |Duration_t-api|      | |c_TimeInfinite-api| |
+-----------------------------------------------------------------------+-----------------------+----------------------+

.. note::
     This QoS Policy concerns to |Topic|, |DataReader| and |DataWriter| entities.
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
mechanism (see |LivelinessQosPolicy-api|).

List of QoS Policy data members:

+------------------------------------------------+--------------------------------+------------------------------------+
| Data Member Name                               | Type                           | Default Value                      |
+================================================+================================+====================================+
| |LivelinessQosPolicy::kind-api|                | :ref:`livelinessqospolicykind` | |AUTOMATIC_LIVELINESS_QOS-api|     |
+------------------------------------------------+--------------------------------+------------------------------------+
| |LivelinessQosPolicy::lease_duration-api|      | |Duration_t-api|               | |c_TimeInfinite-api|               |
+------------------------------------------------+--------------------------------+------------------------------------+
| |LivelinessQosPolicy::announcement_period-api| | |Duration_t-api|               | |c_TimeInfinite-api|               |
+------------------------------------------------+--------------------------------+------------------------------------+

* |LivelinessQosPolicy::kind-api|: This data member establishes if the service needs to assert the liveliness
  automatically or if it needs to wait until the liveliness is asserted by the publishing side.
  See :ref:`livelinessqospolicykind` for further details.
* |LivelinessQosPolicy::lease_duration-api|: Amount of time to wait since the last time the DataWriter asserts its
  liveliness to consider that it is no longer alive.
* |LivelinessQosPolicy::announcement_period-api|: Amount of time between consecutive liveliness messages sent by the
  DataWriter.
  This data member only takes effect if the kind is |AUTOMATIC_LIVELINESS_QOS-api| or
  |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api| and needs to be lower than the
  |LivelinessQosPolicy::lease_duration-api|.

.. note::
     This QoS Policy concerns to |Topic|, |DataReader| and |DataWriter| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`liveliness_compatibilityrule` for further details.

.. _livelinessqospolicykind:

LivelinessQosPolicyKind
"""""""""""""""""""""""

There are three possible values (see |LivelinessQosPolicyKind-api|):

* |AUTOMATIC_LIVELINESS_QOS-api|: The service takes the responsibility for renewing the leases at the required rates, as
  long as the local process where the participant is running and the link connecting it to remote participants exists,
  the entities within the remote participant will be considered alive.
  This kind is suitable for applications that only need to detect whether a remote application is still running.
* The two `Manual` modes require that the application on the publishing side asserts the liveliness periodically
  before the lease_duration timer expires.
  Publishing any new data value implicitly asserts the DataWriter's liveliness, but it can be done explicitly
  by calling the `assert_liveliness` member function.

  * |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|: If one of the entities in the publishing side asserts its liveliness,
    the service deduces that all other entities within the same DomainParticipant are also alive.
  * |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|: This mode is more restrictive and requires that at least one instance within
    the DataWriter is asserted to consider that the DataWriter is alive.

.. _liveliness_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between LivelinessQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
higher or equal to the DataReader kind.
And the order between the different kinds is::

|AUTOMATIC_LIVELINESS_QOS-api| < |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api| < |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|

Table with the possible combinations:

+--------------------------------------------------+--------------------------------------------------+----------------+
| DataWriter kind                                  | DataReader kind                                  | Compatibility  |
+==================================================+==================================================+================+
| |AUTOMATIC_LIVELINESS_QOS-api|                   | |AUTOMATIC_LIVELINESS_QOS-api|                   | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |AUTOMATIC_LIVELINESS_QOS-api|                   | |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | No             |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |AUTOMATIC_LIVELINESS_QOS-api|                   | |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | No             |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | |AUTOMATIC_LIVELINESS_QOS-api|                   | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | No             |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | |AUTOMATIC_LIVELINESS_QOS-api|                   | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api|       | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+
| |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|             | Yes            |
+--------------------------------------------------+--------------------------------------------------+----------------+

Additionally, the |LivelinessQosPolicy::lease_duration-api| of the DataWriter must not be greater than
the |LivelinessQosPolicy::lease_duration-api| of the DataReader.

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
so, how these modifications should be arbitrated (see |OwnershipQosPolicy-api|).

List of QoS Policy data members:

+--------------------------------+-------------------------------+----------------------------------------------+
| Data Member Name               | Type                          | Default Value                                |
+================================+===============================+==============================================+
| |OwnershipQosPolicy::kind-api| | :ref:`ownershipqospolicykind` | |SHARED_OWNERSHIP_QOS-api|                   |
+--------------------------------+-------------------------------+----------------------------------------------+

.. note::
     This QoS Policy concerns to |Topic|, |DataReader| and |DataWriter| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`ownership_compatibilityrule` for further details.

.. _ownershipqospolicykind:

OwnershipQosPolicyKind
""""""""""""""""""""""

There are two possible values (see |OwnershipQosPolicyKind-api|):

* |SHARED_OWNERSHIP_QOS-api|: This option indicates that the service does not enforce unique ownership for each
  instance.
  In this case, multiple DataWriters are allowed to update the same data instance and all the updates are made
  available to the existing DataReaders.
  Those updates are also subject to the :ref:`timebasedfilterqospolicy` or :ref:`historyqospolicy` settings, so they
  can be filtered.
* |EXCLUSIVE_OWNERSHIP_QOS-api|: This option indicates that each instance can only be updated by one DataWriter, meaning
  that at any point in time a single DataWriter owns each instance and is the only one whose modifications will be
  visible for the existing DataReaders.
  The owner can be changed dynamically according to the highest `strength` between the alive DataWriters, which has not
  violated the deadline contract concerning the data instances.
  That `strength` can be changed using the :ref:`ownershipstrengthqospolicy`.
  In case two DataWriters have the same `strength` value,
  the DataWriter with minimum `GUID` value would be the owner of the topic.

.. _ownership_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between OwnershipQosPolicy in |DataReaders| and |DataWriters|, the DataWriter kind must be
equal to the DataReader kind.

Table with the possible combinations:

+--------------------------------------------+-----------------------------------------------+-----------------+
| DataWriter kind                            | DataReader kind                               | Compatibility   |
+============================================+===============================================+=================+
| |SHARED_OWNERSHIP_QOS-api|                 | |SHARED_OWNERSHIP_QOS-api|                    | Yes             |
+--------------------------------------------+-----------------------------------------------+-----------------+
| |SHARED_OWNERSHIP_QOS-api|                 | |EXCLUSIVE_OWNERSHIP_QOS-api|                 | No              |
+--------------------------------------------+-----------------------------------------------+-----------------+
| |EXCLUSIVE_OWNERSHIP_QOS-api|              | |SHARED_OWNERSHIP_QOS-api|                    | No              |
+--------------------------------------------+-----------------------------------------------+-----------------+
| |EXCLUSIVE_OWNERSHIP_QOS-api|              | |EXCLUSIVE_OWNERSHIP_QOS-api|                 | Yes             |
+--------------------------------------------+-----------------------------------------------+-----------------+

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
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_OWNERSHIP
    :end-before: <!--><-->

.. _ownershipstrengthqospolicy:

OwnershipStrengthQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy specifies the value of the `strength` used to arbitrate among multiple DataWriters that attempt to
modify the same data instance. It is only applicable if the :ref:`ownershipqospolicy` kind is set to
|EXCLUSIVE_OWNERSHIP_QOS-api|.
See |OwnershipStrengthQosPolicy-api|.

List of QoS Policy data members:

+-----------------------------------------------------------------+-------------------------------+-------------------+
| Data Member Name                                                | Type                          | Default Value     |
+=================================================================+===============================+===================+
| |OwnershipStrengthQosPolicy::value-api|                         | ``uint32_t``                  | 0                 |
+-----------------------------------------------------------------+-------------------------------+-------------------+

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
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_OWNERSHIP_STRENGTH
    :end-before: <!--><-->


.. _partitionqospolicy:

PartitionQosPolicy
^^^^^^^^^^^^^^^^^^

This Qos Policy allows the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match, but also they have to share
at least one logical partition (see |PartitionQosPolicy-api|).

The empty string is also considered as a valid partition and it matches with other partition names using the same rules
of string matching and regular-expression matching used for any other partition name.

List of QoS Policy data members:

+---------------------------------------+---------------------------------------------+--------------------------------+
| Data Member Name                      | Type                                        | Default Value                  |
+=======================================+=============================================+================================+
| |PartitionQosPolicy::max_size-api|    | ``uint32_t``                                | 0 (Length Unlimited)           |
+---------------------------------------+---------------------------------------------+--------------------------------+
| |PartitionQosPolicy::names-api|       | |SerializedPayload_t-api|                   | Empty List                     |
+---------------------------------------+---------------------------------------------+--------------------------------+

* |PartitionQosPolicy::max_size-api|: Maximum size for the list of partition names.
* |PartitionQosPolicy::names-api|: List of partition names.

.. note::
     This QoS Policy concerns to Publisher and Subscriber entities.
     :raw-html:`<br />`
     Partitions can also be explicitly defined at the endpoint level to override this configuration. Information
     to do so can be found :ref:`here<property_policies_partitions>`.
     :raw-html:`<br />`
     It can be changed on enabled Publishers and Subscribers.

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
See |PresentationQosPolicy-api|.

List of QoS Policy data members:

+----------------------------------+---------------------------------------------+-------------------------------------+
| Data Member Name                 | Type                                        | Default Value                       |
+==================================+=============================================+=====================================+
| |access_scope-api|               | :ref:`presentationqospolicyaccessscopekind` | |INSTANCE_PRESENTATION_QOS-api|     |
+----------------------------------+---------------------------------------------+-------------------------------------+
| |coherent_access-api|            | ``bool``                                    | ``false``                           |
+----------------------------------+---------------------------------------------+-------------------------------------+
| |ordered_access-api|             | ``bool``                                    | ``false``                           |
+----------------------------------+---------------------------------------------+-------------------------------------+

* |access_scope-api|: Determines the largest scope spanning the entities for which the order and coherency can be
  preserved.
  See :ref:`presentationqospolicyaccessscopekind` for further details.
* |coherent_access-api|: Controls whether the service will preserve grouping of changes made on the publishing side,
  such that they are received as a unit on the subscribing side.
* |ordered_access-api|: Controls whether the service supports the ability of the subscriber to see changes in the same
  order as they occurred on the publishing side.

.. note::
     This QoS Policy concerns to |Publisher| and |Subscriber| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`presentation_compatibilityrule`
    for further details.

.. _presentationqospolicyaccessscopekind:

PresentationQosPolicyAccessScopeKind
""""""""""""""""""""""""""""""""""""

There are three possible values, which have different behaviors depending on the values of coherent_access and
ordered_access variables (see |PresentationQosPolicyAccessScopeKind-api|):

* |INSTANCE_PRESENTATION_QOS-api|: The changes to a data instance do not need to be coherent nor ordered with respect to
  the changes to any other instance, which means that the order and coherent changes apply to each instance separately.

  * Enabling the `coherent_access`, in this case, has no effect on how the subscriber can access the data as the scope
    is limited to each instance, changes to separate instances are considered independent and thus cannot be grouped
    by a coherent change.
  * Enabling the `ordered_access`, in this case, only affects to the changes within the same instance.
    Therefore, the changes made to two instances are not necessarily seen in the order they occur even if the same
    application thread and DataWriter made them.

* |TOPIC_PRESENTATION_QOS-api|: The scope spans to all the instances within the same DataWriter.

  * Enabling the `coherent_access` makes that the grouping made with changes within the same DataWriter will be
    available as coherent with respect to other changes to instances in that DataWriter, but will not be grouped with
    changes made to instances belonging to different DataWriters.
  * Enabling the `ordered_access` means that the changes made by a single DataWriter are made available to the
    subscribers in the same order that they occur, but the changes made to instances through different DataWriters are
    not necessarily seen in order.

* |GROUP_PRESENTATION_QOS-api|: The scope spans to all the instances belonging to DataWriters within the same Publisher.

  * Enabling the `coherent_access`, means that the coherent changes made to instances through DataWriters attached to a
    common Publisher are made available as a unit to remote subscribers.
  * Enabling the `ordered_access` with this scope makes that the changes done by any of the DataWriters attached to the
    same Publisher are made available to the subscribers in the same order they occur.

.. _presentation_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between PresentationQosPolicy in DataReaders and DataWriters, the Publisher
|access_scope-api| must be higher or equal to the Subscriber |access_scope-api|.
And the order between the different access scopes is::

|INSTANCE_PRESENTATION_QOS-api| < |TOPIC_PRESENTATION_QOS-api| < |GROUP_PRESENTATION_QOS-api|

Table with the possible combinations:

+---------------------------------------------+------------------------------------------------+-----------------+
| Publisher scope                             | Subscriber scope                               | Compatibility   |
+=============================================+================================================+=================+
| |INSTANCE_PRESENTATION_QOS-api|             | |INSTANCE_PRESENTATION_QOS-api|                | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+
| |INSTANCE_PRESENTATION_QOS-api|             | |TOPIC_PRESENTATION_QOS-api|                   | No              |
+---------------------------------------------+------------------------------------------------+-----------------+
| |INSTANCE_PRESENTATION_QOS-api|             | |GROUP_PRESENTATION_QOS-api|                   | No              |
+---------------------------------------------+------------------------------------------------+-----------------+
| |TOPIC_PRESENTATION_QOS-api|                | |INSTANCE_PRESENTATION_QOS-api|                | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+
| |TOPIC_PRESENTATION_QOS-api|                | |TOPIC_PRESENTATION_QOS-api|                   | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+
| |TOPIC_PRESENTATION_QOS-api|                | |GROUP_PRESENTATION_QOS-api|                   | No              |
+---------------------------------------------+------------------------------------------------+-----------------+
| |GROUP_PRESENTATION_QOS-api|                |  |INSTANCE_PRESENTATION_QOS-api|               | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+
| |GROUP_PRESENTATION_QOS-api|                | |TOPIC_PRESENTATION_QOS-api|                   | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+
| |GROUP_PRESENTATION_QOS-api|                | |GROUP_PRESENTATION_QOS-api|                   | Yes             |
+---------------------------------------------+------------------------------------------------+-----------------+

Additionally, the coherent_access and ordered_access of the Subscriber can only be enabled if they are also enabled on
the Publisher.

.. _readerdatalifecycleqospolicy:

ReaderDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the |DataReader| with respect to the lifecycle of the data instances it
manages, that is, the instances that have been received and for which the DataReader maintains some internal resources.
The DataReader maintains the samples that have not been taken by the application, subject to the constraints imposed by
:ref:`historyqospolicy` and :ref:`resourcelimitsqospolicy`.
See |ReaderDataLifecycleQosPolicy-api|.

Under normal circumstances, the DataReader can only reclaim the resources associated with data instances if there are
no writers and all the samples have been taken. But this fact can cause problems if the application does not take those
samples as the service will prevent the DataReader from reclaiming the resources and they will remain in the DataReader
indefinitely. This QoS exist to avoid that situation.

List of QoS Policy data members:

+----------------------------------------------------------------------------+------------------+----------------------+
| Data Member Name                                                           | Type             | Default Value        |
+============================================================================+==================+======================+
| |ReaderDataLifecycleQosPolicy::autopurge_no_writer_samples_delay-api|      | |Duration_t-api| | |c_TimeInfinite-api| |
+----------------------------------------------------------------------------+------------------+----------------------+
| |ReaderDataLifecycleQosPolicy::autopurge_disposed_samples_delay-api|       | |Duration_t-api| | |c_TimeInfinite-api| |
+----------------------------------------------------------------------------+------------------+----------------------+

* |ReaderDataLifecycleQosPolicy::autopurge_no_writer_samples_delay-api|:
  Defines the maximum duration the DataReader must retain the information
  regarding an instance once its |SampleInfo::instance_state-api| becomes |NOT_ALIVE_NO_WRITERS_INSTANCE_STATE-api|.
  After this time elapses, the DataReader purges all the internal information of the instance, including the untaken
  samples that will be lost.
* |ReaderDataLifecycleQosPolicy::autopurge_disposed_samples_delay-api|:
  Defines the maximum duration the DataReader must retain the information
  regarding an instance once its |SampleInfo::instance_state-api| becomes |NOT_ALIVE_DISPOSED_INSTANCE_STATE-api|.
  After this time elapses, the DataReader purges all the samples for the instance.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.


.. _reliabilityqospolicy:

ReliabilityQosPolicy
^^^^^^^^^^^^^^^^^^^^

This QoS Policy indicates the level of reliability offered and requested by the service.
See |ReliabilityQosPolicy-api|.

List of QoS Policy data members:

+-----------------------------------------------+---------------------------------+------------------------------------+
| Data Member Name                              | Type                            | Default Value                      |
+===============================================+=================================+====================================+
|                                               |                                 | |BEST_EFFORT_RELIABILITY_QOS-api|  |
|                                               |                                 | for DataReaders                    |
| |ReliabilityQosPolicy::kind-api|              | :ref:`reliabilityqospolicykind` | :raw-html:`<br />`                 |
|                                               |                                 | |RELIABLE_RELIABILITY_QOS-api|     |
|                                               |                                 | for DataWriters                    |
+-----------------------------------------------+---------------------------------+------------------------------------+
| |ReliabilityQosPolicy::max_blocking_time-api| | |Duration_t-api|                | 100 ms                             |
+-----------------------------------------------+---------------------------------+------------------------------------+

* |ReliabilityQosPolicy::kind-api|: Specifies the behavior of the service regarding delivery of the samples.
  See :ref:`reliabilityqospolicykind` for further details.
* |ReliabilityQosPolicy::max_blocking_time-api|: Configures the maximum duration that the write operation can be
  blocked.

.. note::
     This QoS Policy concerns to |Topic|, |DataWriter| and |DataReader| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. important::
    Setting this QoS Policy to |BEST_EFFORT_RELIABILITY_QOS-api| affects to the :ref:`durabilityqospolicy`, making the
    endpoints behave as |VOLATILE_DURABILITY_QOS-api|.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`reliability_compatibilityrule` for further details.

.. _reliabilityqospolicykind:

ReliabilityQosPolicyKind
""""""""""""""""""""""""

There are two possible values ():

* |BEST_EFFORT_RELIABILITY_QOS-api|: It indicates that it is acceptable not to retransmit the missing samples, so the
  messages are sent without waiting for an arrival confirmation.
  Presumably new values for the samples are generated often enough that it is not necessary to re-send any sample.
  However, the data samples sent by the same DataWriter will be stored in the DataReader history in the same order they
  occur.
  In other words, even if the DataReader misses some data samples, an older value will never overwrite a newer value.
* |RELIABLE_RELIABILITY_QOS-api|: It indicates that the service will attempt to deliver all samples of the DataWriter's
  history expecting an arrival confirmation from the DataReader.
  The data samples sent by the same DataWriter cannot be made available to the DataReader if there are previous samples
  that have not been received yet.
  The service will retransmit the lost data samples in order to reconstruct a correct snapshot of the DataWriter
  history before it is accessible by the DataReader.

  This option may block the write operation, hence the |ReliabilityQosPolicy::max_blocking_time-api| is set that
  will unblock it once the time expires.
  But if the |ReliabilityQosPolicy::max_blocking_time-api| expires before the data is sent, the write operation will
  return an error.

.. _reliability_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between ReliabilityQosPolicy in DataReaders and DataWriters, the DataWriter kind
must be higher or equal to the DataReader kind.
And the order between the different kinds is::

|BEST_EFFORT_RELIABILITY_QOS-api| < |RELIABLE_RELIABILITY_QOS-api|

Table with the possible combinations:

+---------------------------------------------+---------------------------------------------+-----------------+
| DataWriter kind                             | DataReader kind                             | Compatibility   |
+=============================================+=============================================+=================+
| |BEST_EFFORT_RELIABILITY_QOS-api|           | |BEST_EFFORT_RELIABILITY_QOS-api|           | Yes             |
+---------------------------------------------+---------------------------------------------+-----------------+
| |BEST_EFFORT_RELIABILITY_QOS-api|           | |RELIABLE_RELIABILITY_QOS-api|              | No              |
+---------------------------------------------+---------------------------------------------+-----------------+
| |RELIABLE_RELIABILITY_QOS-api|              | |BEST_EFFORT_RELIABILITY_QOS-api|           | Yes             |
+---------------------------------------------+---------------------------------------------+-----------------+
| |RELIABLE_RELIABILITY_QOS-api|              | |RELIABLE_RELIABILITY_QOS-api|              | Yes             |
+---------------------------------------------+---------------------------------------------+-----------------+

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
See |ResourceLimitsQosPolicy-api|.

List of QoS Policy data members:

+----------------------------------------------------------------------------------------+-------------+---------------+
| Data Member Name                                                                       | Type        | Default Value |
+========================================================================================+=============+===============+
| |ResourceLimitsQosPolicy::max_samples-api|                                             | ``int32_t`` | 5000          |
+----------------------------------------------------------------------------------------+-------------+---------------+
| |ResourceLimitsQosPolicy::max_instances-api|                                           | ``int32_t`` | 10            |
+----------------------------------------------------------------------------------------+-------------+---------------+
| |ResourceLimitsQosPolicy::max_samples_per_instance-api|                                | ``int32_t`` | 400           |
+----------------------------------------------------------------------------------------+-------------+---------------+
| |ResourceLimitsQosPolicy::allocated_samples-api|                                       | ``int32_t`` | 100           |
+----------------------------------------------------------------------------------------+-------------+---------------+
| |ResourceLimitsQosPolicy::extra_samples-api|                                           | ``int32_t`` | 1             |
+----------------------------------------------------------------------------------------+-------------+---------------+

* |ResourceLimitsQosPolicy::max_samples-api|: Controls the maximum number of samples that the DataWriter or DataReader
  can manage across all the
  instances associated with it.
  In other words, it represents the maximum samples that the middleware can store for a DataReader or DataWriter.
* |ResourceLimitsQosPolicy::max_instances-api|: Controls the maximum number of instances that a DataWriter or
  DataReader can manage.
* |ResourceLimitsQosPolicy::max_samples_per_instance-api|: Controls the maximum number of samples within an instance
  that the DataWriter or
  DataReader can manage.
* |ResourceLimitsQosPolicy::allocated_samples-api|: States the number of samples that will be allocated on
  initialization.
* |ResourceLimitsQosPolicy::extra_samples-api|: States the number of extra samples that will be allocated on
  the pool, so the maximum number of samples on the pool will be
  |ResourceLimitsQosPolicy::max_samples-api| plus |ResourceLimitsQosPolicy::extra_samples-api|.
  These extra samples act as a reservoir of samples even when the history is full.

.. note::
     This QoS Policy concerns to Topic, DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _resourcelimits_consistencyrule:

Consistency Rule
""""""""""""""""

To maintain the consistency within the ResourceLimitsQosPolicy, the values of the data members must follow the next
conditions:

* The value of |ResourceLimitsQosPolicy::max_samples-api| must be higher or equal to the value of
  |ResourceLimitsQosPolicy::max_samples_per_instance-api|.
* The value established for the :ref:`historyqospolicy` |HistoryQosPolicy::depth-api| must be lower or equal to the
  value stated for
  |ResourceLimitsQosPolicy::max_samples_per_instance-api|.


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

Filter that allows a |DataReader| to specify that it is interested only in a subset of the values of the data.
This filter states that the DataReader does not want to receive more than one value each
|TimeBasedFilterQosPolicy::minimum_separation-api|, regardless of how fast the changes occur.
See |TimeBasedFilterQosPolicy-api|.

The |TimeBasedFilterQosPolicy::minimum_separation-api| must be lower than the :ref:`deadlineqospolicy`
|DeadlineQosPolicy::period-api|.
By default, the |TimeBasedFilterQosPolicy::minimum_separation-api| is zero, which means that the DataReader is
potentially interested in all the values.

List of QoS Policy data members:

+---------------------------------------------------------------------+-----------------------------+------------------+
| Data Member Name                                                    | Type                        | Default Value    |
+=====================================================================+=============================+==================+
||TimeBasedFilterQosPolicy::minimum_separation-api|                   | |Duration_t-api|            | |c_TimeZero-api| |
+---------------------------------------------------------------------+-----------------------------+------------------+

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

.. _topicdataqospolicy:

TopicDataQosPolicy
^^^^^^^^^^^^^^^^^^

Allows the application to attach additional information to a created |Topic| so that when it is discovered by a remote
application, it can access the data and use it.
See |TopicDataQosPolicy-api|.

List of QoS Policy data members:

+------------------------------------------------------------+----------------------------------------+----------------+
| Data Member Name                                           | Type                                   | Default Value  |
+============================================================+========================================+================+
| collection                                                 | std::vector<|octet-api|>               | Empty vector   |
+------------------------------------------------------------+----------------------------------------+----------------+

.. note::
    This QoS Policy concerns to Topic entities.
    :raw-html:`<br />`
    It can be changed even if it is already created.


Example
"""""""

.. tabs::

  .. tab:: C++

    .. literalinclude:: ../../../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 8
       :start-after: //DDS_CHANGE_TOPIC_DATA_QOS_POLICY
       :end-before: //!

  .. tab:: XML

    .. literalinclude:: ../../../../../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_CHANGE_TOPIC_DATA_QOS_POLICY
       :end-before: <!--><-->

.. _transportpriorityqospolicy:

TransportPriorityQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

The purpose of this QoS Policy is to allow the service to take advantage of those transports capable of sending
messages with different priorities. It establishes the priority of the underlying transport used to send the data.
See |TransportPriorityQosPolicy-api|

You can choose any value within the 32-bit range for the priority. The higher the value, the higher the priority.

List of QoS Policy data members:

+-------------------------------------------------------------------------------------+----------------+---------------+
| Data Member Name                                                                    | Type           | Default Value |
+=====================================================================================+================+===============+
| |TransportPriorityQosPolicy::value-api|                                             | ``uint32_t``   | 0             |
+-------------------------------------------------------------------------------------+----------------+---------------+

.. note::
     This QoS Policy concerns to |Topic| and |DataWriter| entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.


.. _userdataqospolicy:

UserDataQosPolicy
^^^^^^^^^^^^^^^^^

Allows the application to attach additional information to the |Entity| object so that when the entity is discovered
the remote application can access the data and use it.
For example, it can be used to attach the security credentials to authenticate the source from the remote application.
See |UserDataQosPolicy-api|.

List of QoS Policy data members:

+-------------------------------------------------------------------+----------------------------------+---------------+
| Data Member Name                                                  | Type                             | Default Value |
+===================================================================+==================================+===============+
| collection                                                        | std::vector<|octet-api|>         | Empty vector  |
+-------------------------------------------------------------------+----------------------------------+---------------+

.. note::
    This QoS Policy concerns to all DDS entities.
    :raw-html:`<br />`
    It can be changed on enabled entities.

Example
"""""""

.. tabs::

  .. tab:: C++

    .. literalinclude:: ../../../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 8
       :start-after: //DDS_CHANGE_USER_DATA_QOS_POLICY
       :end-before: //!

  .. tab:: XML

    .. literalinclude:: ../../../../../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_CHANGE_USER_DATA_QOS_POLICY
       :end-before: <!--><-->

.. _writerdatalifecycleqospolicy:

WriterDataLifecycleQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataWriter with respect to the lifecycle of the data instances it manages
, that is, the instance that has been either explicitly registered with the DataWriter using the register operations
or implicitly by directly writing data.

The |WriterDataLifecycleQosPolicy::autodispose_unregistered_instances-api|
controls whether a DataWriter will automatically dispose an instance each time
it is unregistered. Even if it is disabled, the application can still get the same result if it uses the dispose
operation before unregistering the instance.

List of QoS Policy data members:

+--------------------------------------------------------------------------------------+-----------+-------------------+
| Data Member Name                                                                     | Type      | Default Value     |
+======================================================================================+===========+===================+
| |WriterDataLifecycleQosPolicy::autodispose_unregistered_instances-api|               | ``bool``  | ``true``          |
+--------------------------------------------------------------------------------------+-----------+-------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It can be changed on enabled entities.

