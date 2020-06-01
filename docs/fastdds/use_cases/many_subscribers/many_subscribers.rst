.. _use-case-manySubscribers:

Topics with many subscribers
============================

By default, every time a :ref:`dds_layer_publisher_dataWriter` publishes a data change on a
:ref:`Topics<dds_layer_topic_topic>`, it sends a unicast message for every
:ref:`dds_layer_subscriber_dataReader` that is subscribed to the :ref:`Topics<dds_layer_topic_topic>`.
If there are several :ref:`DataReaders<dds_layer_subscriber_dataReader>` subscribed, it is recommendable
to use multicast instead of unicast.
By doing so, only one network package will be sent for each sample.
This will improve both CPU and network usage.

The example below shows how to set a :ref:`dds_layer_publisher_dataWriterQos` to configure
a :ref:`dds_layer_publisher_dataWriter` to use a multicast transport.
More information about configuring local and remote locators on endpoints can be found in :ref:`rtpsendpointqos`.

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
   +-------------------------------------------------------+


