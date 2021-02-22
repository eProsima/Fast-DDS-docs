.. _use-case-manySubscribers:

Topics with many subscribers
============================

By default, every time a :ref:`dds_layer_publisher_dataWriter` publishes a data change on a
:ref:`Topic<dds_layer_topic_topic>`, it sends a unicast message for every
:ref:`dds_layer_subscriber_dataReader` that is subscribed to the Topic.
If there are several DataReaders subscribed, it is recommendable
to use multicast instead of unicast.
By doing so, only one network package will be sent for each sample.
This will improve both CPU and network usage.

This solution can be implemented with :ref:`transport_udp_udp` or :ref:`transport_sharedMemory_sharedMemory` (SHM).
SHM transport is multicast by default, but is only available between DataWriters and
DataReaders on the same machine.
UDP transport needs some extra configuration.
The example below shows how to set a :ref:`dds_layer_subscriber_dataReaderQos` to configure
a DataReader to use a multicast transport on UDP.
More information about configuring local and remote locators on endpoints can be found in :ref:`rtpsendpointqos`.

.. note::

   Multicast over UDP can be problematic on some scenarios, mainly WiFi and complex networks
   with multiple network links.

+-------------------------------------------------------+
| **C++**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp        |
|    :language: c++                                     |
|    :start-after: //DDS_MULTICAST_DELIVERY             |
|    :end-before: //!--                                 |
|    :dedent: 8                                         |
+-------------------------------------------------------+
| **XML**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml            |
|    :language: xml                                     |
|    :start-after: <!-->DDS_MULTICAST_DELIVERY          |
|    :end-before: <!--><-->                             |
|    :lines: 2-3,5-                                     |
|    :append: </profiles>                               |
+-------------------------------------------------------+


