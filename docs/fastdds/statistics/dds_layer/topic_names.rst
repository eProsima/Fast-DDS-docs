.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _statistics_topic_names:

Statistics Topic names
======================

Data collected by the *Fast DDS Statistics module* is published in one of the topics listed below.
In order to simplify its use, the API provides aliases for the different statistics topics (see
:ref:`api_statistics_topic_names`).
The following table shows the correlation between the topic name and the corresponding alias.

+-------------------------------------------------+--------------------------------------------------------------------+
| **Topic name**                                  | **Alias**                                                          |
+=================================================+====================================================================+
| ``_fastdds_statistics_history2history_latency`` | |HISTORY_LATENCY_TOPIC|                                            |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_network_latency``         | |NETWORK_LATENCY_TOPIC|                                            |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_publication_throughput``  | |PUBLICATION_THROUGHPUT_TOPIC|                                     |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_subscription_throughput`` | |SUBSCRIPTION_THROUGHPUT_TOPIC|                                    |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_rtps_sent``               | |RTPS_SENT_TOPIC|                                                  |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_rtps_lost``               | |RTPS_LOST_TOPIC|                                                  |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_heartbeat_count``         | |HEARTBEAT_COUNT_TOPIC|                                            |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_acknack_count``           | |ACKNACK_COUNT_TOPIC|                                              |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_nackfrag_count``          | |NACKFRAG_COUNT_TOPIC|                                             |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_gap_count``               | |GAP_COUNT_TOPIC|                                                  |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_data_count``              | |DATA_COUNT_TOPIC|                                                 |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_resent_datas``            | |RESENT_DATAS_TOPIC|                                               |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_sample_datas``            | |SAMPLE_DATAS_TOPIC|                                               |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_pdp_packets``             | |PDP_PACKETS_TOPIC|                                                |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_edp_packets``             | |EDP_PACKETS_TOPIC|                                                |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_discovered_entity``       | |DISCOVERY_TOPIC|                                                  |
+-------------------------------------------------+--------------------------------------------------------------------+
| ``_fastdds_statistics_physical_data``           | |PHYSICAL_DATA_TOPIC|                                              |
+-------------------------------------------------+--------------------------------------------------------------------+

|HISTORY_LATENCY_TOPIC|
-----------------------

The ``_fastdds_statistics_history2history_latency`` statistics topic collects data related with the latency between any
two matched endpoints.
This measurement provides information about the DDS overall latency independent of the user's application overhead.
Specifically, the measured latency corresponds to the time spent between the instant when the sample is written to the
DataWriter's history and the time when the sample is added to the DataReader's history and the notification is issued to
the corresponding user's callback.

|NETWORK_LATENCY_TOPIC|
-----------------------

The ``_fastdds_statistics_network_latency`` statistics topic collects data related with the network latency (expressed
in *ns*) between any two communicating locators.
This measurement provides information about the transport layer latency.
The measured latency corresponds to the time spent between the message being written until the message being received
in the |MessageReceiver|.

.. important::
    In the case of :ref:`transport_tcp_tcp`, the reported latency also includes the time spent on the datagram's CRC
    related operations.
    Mind that is possible to disable CRC operations when defining the :ref:`transport_tcp_transportDescriptor`.

|PUBLICATION_THROUGHPUT_TOPIC|
------------------------------

The ``_fastdds_statistics_publication_throughput`` statistics topic collects the amount of data (expressed in *B/s*)
that is being sent by each DataWriter.
This measurement provides information about the publication's throughput.

|SUBSCRIPTION_THROUGHPUT_TOPIC|
-------------------------------

The ``_fastdds_statistics_subscription_throughput`` statistics topic collects the amount of data (expressed in *B/s*)
that is being received by each DataReader.
This measurement provides information about the subscription's throughput.

|RTPS_SENT_TOPIC|
-----------------

The ``_fastdds_statistics_rtps_sent`` statistics topic collects the number of RTPS packets and bytes that are being sent
from each DDS entity to each locator.

|RTPS_LOST_TOPIC|
-----------------

The ``_fastdds_statistics_rtps_lost`` statistics topic collects the number of RTPS packets and bytes that are being lost
in the transport layer (dropped somewhere in between) in the communication between each DDS entity and locator.

|HEARTBEAT_COUNT_TOPIC|
-----------------------

The ``_fastdds_statistics_heartbeat_count`` statistics topic collects the number of heartbeat messages sent by each
user's DataWriter.
This topic does not apply to builtin (related to :ref:`discovery`) and statistics DataWriters.
Heartbeat messages are only sent if the |ReliabilityQosPolicy| is set to |RELIABLE_RELIABILITY_QOS-api|.
These messages report the DataWriter's status.

|ACKNACK_COUNT_TOPIC|
---------------------

The ``_fastdds_statistics_acknack_count`` statistics topic collects the number of acknack messages sent by each user's
DataReader.
This topic does not apply to builtin DataReaders (related to :ref:`discovery`).
Acknack messages are only sent if the |ReliabilityQosPolicy| is set to |RELIABLE_RELIABILITY_QOS-api|.
These messages report the DataReader's status.

|NACKFRAG_COUNT_TOPIC|
----------------------

The ``_fastdds_statistics_nackfrag_count`` statistics topic collects the number of nackfrag messages sent by each user's
DataReader.
This topic does not apply to builtin DataReaders (related to :ref:`discovery`).
Nackfrag messages are only sent if the |ReliabilityQosPolicy| is set to |RELIABLE_RELIABILITY_QOS-api|.
These messages report the data fragments that have not been received yet by the DataReader.

|GAP_COUNT_TOPIC|
-----------------

The ``_fastdds_statistics_gap_count`` statistics topic collects the number of gap messages sent by each user's
DataWriter.
This topic does not apply to builtin (related to :ref:`discovery`) and statistics DataWriters.
Gap messages are only sent if the |ReliabilityQosPolicy| is set to |RELIABLE_RELIABILITY_QOS-api|.
These messages report that some specific samples are not relevant to a specific DataReader.

|DATA_COUNT_TOPIC|
------------------

The ``_fastdds_statistics_data_count`` statistics topic collects the total number of user's data messages and data
fragments (in case that the message size is large enough to require RTPS fragmentation) that have been sent by each
user's DataWriter.
This topic does not apply to builtin (related to :ref:`discovery`) and statistics DataWriters.

|RESENT_DATAS_TOPIC|
--------------------

The ``_fastdds_statistics_resent_data`` statistics topic collects the total number of user's data messages and data
fragments (in case that the message size is large enough to require RTPS fragmentation) that have been necessary to
resend by each user's DataWriter.
This topic does not apply to builtin (related to :ref:`discovery`) and statistics DataWriters.

|SAMPLE_DATAS_TOPIC|
--------------------

The ``_fastdds_statistics_sample_datas`` statistics topic collects the number of user's data messages (or data fragments
in case that the message size is large enough to require RTPS fragmentation) that have been sent by the user's
DataWriter to completely deliver a single sample.
This topic does not apply to builtin (related to :ref:`discovery`) and statistics DataWriters.

|PDP_PACKETS_TOPIC|
-------------------

The ``_fastdds_statistics_pdp_packets`` statistics topic collects the number of PDP discovery traffic RTPS packets
transmitted by each DDS |DomainParticipant-api|.
PDP packets are the data messages exchanged during the PDP discovery phase (see :ref:`disc_phases` for more
information).

|EDP_PACKETS_TOPIC|
-------------------

The ``_fastdds_statistics_edp_packets`` statistics topic collects the number of EDP discovery traffic RTPS packets
transmitted by each DDS |DomainParticipant-api|.
EDP packets are the data messages exchanged during the EDP discovery phase (see :ref:`disc_phases` for more
information).

.. _statistics_topic_names_discovery:

|DISCOVERY_TOPIC|
-----------------

The ``_fastdds_statistics_discovered_entity`` statistics topic reports the time when each local |DomainParticipant-api|
discovers any remote DDS entity (with the exception of those DDS entities related with the *Fast DDS Statistics
module*).
This topic also carries the |PHYSICAL_DATA_TOPIC| information for the case of discovered |DomainParticipant-api|; if the
discovered entity is either a |DataReader-api| or |DataWriter-api|, then the physical information is empty (see
:ref:`property_policies_physical_data` for more information about how to configure the physical data conveyed on the
discovery messages).

.. _statistics_topic_names_physical:

|PHYSICAL_DATA_TOPIC|
---------------------

The ``_fastdds_statistics_physical_data`` statistics topic reports the host, user and process where the
*Fast DDS Statistics module* is running.
