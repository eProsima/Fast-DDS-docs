Publisher-Subscriber Layer
==========================

*eProsima Fast RTPS* provides a high level Publisher-Subscriber Layer, which is a simple to use abstraction over the RTPS protocol.
By using this layer, you can code a straight-to-the-point application while letting the library take care of the lower level configuration.

How to use the Publisher-Subscriber Layer
-----------------------------------------

We are going to use the example built in the previous section to explain how this layer works.

The first step is to create a :class:`Participant` instance, which will act as a container for the Publishers and
Subscribers our application needs. For this we use :class:`Domain`, a static class that manages RTPS entities.
We also need to pass a configuration structure for the Participant, which can be left in its default configuration for now:

.. code-block:: c++

   ParticipantAttributes participant_attr; //Configuration structure
   Participant *participant = Domain::createParticipant(participant_attr);

The default configuration provides a basic working set of options with predefined ports for communications.
During this tutorial you will learn to tune *eProsima Fast RTPS*.

In order to use our topic, we have to register it within the :class:`Participant` using the code generated with *fastrtpsgen*
(see :ref:`fastrtpsgen-intro`. Once again, this is done by using the :class:`Domain` class:

.. code-block:: c++

   HelloWorldPubSubType m_type; //Auto-generated type from FastRTPSGen
   Domain::registerType(participant, &m_type);

Once set up, we instantiate a :class:`Publisher` within our :class:`Participant`:

.. code-block:: c++

   PublisherAttributes publisher_attr; //Configuration structure
   PubListener m_listener; //Class that implements callbacks from the publisher
   Publisher *publisher = Domain::createPublisher(participant, publisher_attr, (PublisherListener *)&m_listener);

Once the :class:`Publisher` is functional, posting data is a simple process:

.. code-block:: c++

   HelloWorld m_Hello; //Auto-generated container class for topic data from FastRTPSGen
   m_Hello.msg("Hello there!"); // Add contents to the message
   publisher->write((void *)&m_Hello); //Publish
	
The :class:`Publisher` has a set of optional callback functions that are triggered when events happen. An example is when a :class:`Subscriber` starts listening to our topic.

To implement these callbacks we create the class :class:`PubListener`, which inherits from the base class :class:`PublisherListener`.
We pass an instance to this class during the creation of the :class:`Publisher`.

.. code-block:: c++

    class PubListener : public PublisherListener
    {
        public PubListener(){};
        ~PubListener(){};
        void onPublicationmatched(Publisher* pub, MatchingInfo& info)
        {
            //Callback implementation. This is called each time the Publisher finds a Subscriber on the network that listens to the same topic.
        }
    } m_listener;

The :class:`Subscriber` creation and implementation is symmetric.

.. code-block:: c++

    SubscriberAttributes subscriber_attr; //Configuration structure
    SubListener m_listener; //Class that implements callbacks from the Subscriber
    Subscriber *subscriber = Domain::createSubscriber(participant,subscriber_attr,(SubsciberListener*)&m_listener);

Incoming messages are processed within the callback that is called when a new message is received:

.. code-block c++

    class SubListener: public SubscriberListener
    {
    public:
        SubListener(){};
        ~SubListener(){};
        HelloWorld m_Hello; //Storage for incoming messages
        SampleInfo_t m_info; //Auxiliary structure with meta-data on the message
        void onNewDataMessage(Subscriber * sub)
        {
            if(sub->takeNextData((void*)&m_Hello, &m_info))
                if(m_info.sampleKind == ALIVE)
                    std::cout << "New message: " << m_Hello.msg() << std::endl;
        }
    }

.. _configuration:

Configuration
-------------

*eProsima Fast RTPS* entities can be configured through the code or XML profiles. This section will show both alternatives.

Participant configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`Participant` can be configured via the :class:`ParticipantAttributes` structure.
``createParticipant`` function accepts an instance of this structure.

.. code-block:: c++

   ParticipantAttributes participant_attr;

   participant_attr.setName("my_participant");
   participant_attr.rtps.builtin.domainId = 80;

   Participant *participant = Domain::createParticipant(participant_attr);

Also it can be configured through an XML profile. ``createParticipant`` function accepts a name of an XML profile.

.. code-block:: c++

   Participant *participant = Domain::createParticipant("participant_xml_profile");

About XML profiles you can learn more in :ref:`xml-profiles`. This is an example of a participant XML profile.

.. code-block:: xml

   <participant profile_name="participant_xml_profile">
       <rtps>
           <name>my_participant</name>
           <builtin>
               <domainId>80</domainId>
           </builtin>
       </rtps>
   </participant>

We will now go over the most common configuration options.

* **Participant name:** the name of the :class:`Participant` forms part of the meta-data of the RTPS protocol.
   
   +------------------------------------------------+------------------------------------------------------------+
   | C++                                            | XML                                                        |
   +================================================+============================================================+
   | .. code-block:: c++                            | .. code-block:: xml                                        |
   |                                                |                                                            |
   |                                                |    <profiles>                                              |
   |    participant_attr.setName("my_participant"); |       <participant profile_name="participant_xml_profile"> |
   |                                                |          <rtps>                                            |
   |                                                |             <name>my_participant</name>                    |
   |                                                |          </rtps>                                           |
   |                                                |       </participant>                                       |
   |                                                |    </profiles>                                             |
   +------------------------------------------------+------------------------------------------------------------+
   
* **DomainId:** Publishers and Subscribers can only talk to each other if their Participants belong to the same DomainId.
   
   +-------------------------------------------------+------------------------------------------------------------+
   | C++                                             | XML                                                        |
   +=================================================+============================================================+
   | .. code-block:: c++                             | .. code-block:: xml                                        |
   |                                                 |                                                            |
   |                                                 |    <profiles>                                              |
   |    participant_attr.rtps.builtin.domainId = 80; |       <participant profile_name="participant_xml_profile"> |
   |                                                 |          <rtps>                                            |
   |                                                 |             <builtin>                                      |
   |                                                 |                <domainId>80</domainId>                     |
   |                                                 |             </builtin>                                     |
   |                                                 |          </rtps>                                           |
   |                                                 |       </participant>                                       |
   |                                                 |    </profiles>                                             |
   +-------------------------------------------------+------------------------------------------------------------+

Publisher and Subscriber configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`Publisher` can be configured via the :class:`PublisherAttributes` structure and
``createPublisher`` function accepts an instance of this structure. The :class:`Subscriber` can be configured via the
:class:`SubscriberAttributes` structure and ``createSubscriber`` function accepts an instance of this structure.

.. code-block:: c++

   PublisherAttributes publisher_attr;
   Publisher *publisher = Domain::createPublisher(participant, publisher_attr);

   SubscriberAttributes subscriber_attr;
   Subscriber *subscriber = Domain::createSubscriber(participant, subscriber_attr);

Also these entities can be configured through an XML profile. ``createPublisher`` and ``createSubscriber`` functions
accept a name of an XML profile.

.. code-block:: c++

   Publisher *publisher = Domain::createPublisher(participant, "publisher_xml_profile");
   Subscriber *subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");

We will now go over the most common configuration options.

Topic information
*****************

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange messages.

   +------------------------------------------------------------+----------------------------------------------------------+
   | C++                                                        | XML                                                      |
   +============================================================+==========================================================+
   | .. code-block:: c++                                        | .. code-block:: xml                                      |
   |                                                            |                                                          |
   |    publisher_attr.topic.topicDataType = "HelloWorldType";  |    <profiles>                                            |
   |    publisher_attr.topic.topicName = "HelloWorldTopic";     |       <publisher profile_name="publisher_xml_profile">   |
   |                                                            |          <topic>                                         |
   |    subscriber_attr.topic.topicDataType = "HelloWorldType"; |             <dataType>HelloWorldType</dataType>          |
   |    subscriber_attr.topic.topicName = "HelloWorldTopic";    |             <name>HelloWorldTopic</name>                 |
   |                                                            |          </topic>                                        |
   |                                                            |       </publisher>                                       |
   |                                                            |                                                          |
   |                                                            |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                                            |          <topic>                                         |
   |                                                            |             <dataType>HelloWorldType</dataType>          |
   |                                                            |             <name>HelloWorldTopic</name>                 |
   |                                                            |          </topic>                                        |
   |                                                            |       </subscriber>                                      |
   |                                                            |    </profiles>                                           |
   +------------------------------------------------------------+----------------------------------------------------------+

.. _reliability:

Reliability
***********

The RTPS standard defines two behaviour modes for message delivery:

   * Best-Effort (default): Messages are sent without arrival confirmation from the receiver (subscriber).
     It is fast, but messages can be lost.

   * Reliable: The sender agent (publisher) expects arrival confirmation from the receiver (subscriber).
     It is slower, but prevents data loss.

   +---------------------------------------------+----------------------------------------------------------+
   | C++                                         | XML                                                      |
   +=============================================+==========================================================+
   | .. code-block:: c++                         | .. code-block:: xml                                      |
   |                                             |                                                          |
   |    publisher_attr.qos.m_reliability.kind =  |    <profiles>                                            |
   |       RELIABLE_RELIABILITY_QOS;             |       <publisher profile_name="publisher_xml_profile">   |
   |                                             |          <qos>                                           |
   |    subscriber_attr.qos.m_reliability.kind = |             <reliability>                                |
   |       BEST_EFFORT_RELIABILITY_QOS;          |                <kind>RELIABLE</kind>                     |
   |                                             |             </reliability>                               |
   |                                             |          </qos>                                          |
   |                                             |       </publisher>                                       |
   |                                             |                                                          |
   |                                             |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                             |          <qos>                                           |
   |                                             |             <reliability>                                |
   |                                             |                <kind>BEST_EFFORT</kind>                  |
   |                                             |             </reliability>                               |
   |                                             |          </qos>                                          |
   |                                             |       </subscriber>                                      |
   |                                             |    </profiles>                                           |
   +---------------------------------------------+----------------------------------------------------------+

   Some reliability combinations make a publisher and a subscriber incompatible and unable to talk to each other.
   Next table shows the incompatibilities.

   +-----------------------------+-----------------+--------------+
   | **Publisher \\ Subscriber** | **Best Effort** | **Reliable** |
   +-----------------------------+-----------------+--------------+
   | **Best Effort**             |  ✓              |  ✕           |
   +-----------------------------+-----------------+--------------+
   | **Reliable**                |  ✓              |  ✓           |
   +-----------------------------+-----------------+--------------+


.. _history-qos:

History
*******

There are two policies for sample storage:

   * Keep-All: Store all samples in memory.

   * Keep-Last (Default): Store samples up to a maximum *depth*.
     When this limit is reached, they start to become overwritten.

   +-----------------------------------------------+----------------------------------------------------------+
   | C++                                           | XML                                                      |
   +===============================================+==========================================================+
   | .. code-block:: c++                           | .. code-block:: xml                                      |
   |                                               |                                                          |
   |    publisher_attr.topic.historyQos.kind =     |    <profiles>                                            |
   |       KEEP_ALL_HISTORY_QOS;                   |       <publisher profile_name="publisher_xml_profile">   |
   |                                               |          <topic>                                         |
   |    subscriber_attr.topic.historyQos.kind =    |             <historyQos>                                 |
   |       KEEP_LAST_HISTORY_QOS;                  |                <kind>KEEP_ALL</kind>                     |
   |    subscriber_attr.topic.historyQos.depth = 5 |             </historyQos>                                |
   |                                               |          </topic>                                        |
   |                                               |       </publisher>                                       |
   |                                               |                                                          |
   |                                               |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                               |          <topic>                                         |
   |                                               |             <historyQos>                                 |
   |                                               |                <kind>KEEP_LAST</kind>                    |
   |                                               |                <depth>5</depth>                          |
   |                                               |             </historyQos>                                |
   |                                               |          </topic>                                        |
   |                                               |       </subscriber>                                      |
   |                                               |    </profiles>                                           |
   +-----------------------------------------------+----------------------------------------------------------+

Durability
**********

Durability configuration of the endpoint defines how it behaves regarding samples that existed on the topic before a
subscriber joins

   * Volatile: Past samples are ignored, a joining subscriber receives samples generated after the moment it matches.
   * Transient Local (Default): When a new subscriber joins, its History is filled with past samples.
   * Transient: When a new subscriber joins, its History is filled with past samples, which are stored on persistent storage (see :ref:`persistence`).

   +--------------------------------------------+----------------------------------------------------------+
   | C++                                        | XML                                                      |
   +============================================+==========================================================+
   | .. code-block:: c++                        | .. code-block:: xml                                      |
   |                                            |                                                          |
   |    publisher_attr.qos.m_durability.kind =  |    <profiles>                                            |
   |       TRANSIENT_LOCAL_DURABILITY_QOS;      |       <publisher profile_name="publisher_xml_profile">   |
   |                                            |          <qos>                                           |
   |    subscriber_attr.qos.m_durability.kind = |             <durability>                                 |
   |       VOLATILE_DURABILITY_QOS;             |                <kind>TRANSIENT_LOCAL</kind>              |
   |                                            |             </durability>                                |
   |                                            |          </qos>                                          |
   |                                            |       </publisher>                                       |
   |                                            |                                                          |
   |                                            |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                            |          <qos>                                           |
   |                                            |             <durability>                                 |
   |                                            |                <kind>VOLATILE</kind>                     |
   |                                            |             </durability>                                |
   |                                            |          </qos>                                          |
   |                                            |       </subscriber>                                      |
   |                                            |    </profiles>                                           |
   +--------------------------------------------+----------------------------------------------------------+

Resource limits
***************

Allow to control the maximum size of the History and other resources.

   +---------------------------------------------------------------+----------------------------------------------------------+
   | C++                                                           | XML                                                      |
   +===============================================================+==========================================================+
   | .. code-block:: c++                                           | .. code-block:: xml                                      |
   |                                                               |                                                          |
   |    publisher_attr.topic.resourceLimitsQos.max_samples = 200;  |    <profiles>                                            |
   |                                                               |       <publisher profile_name="publisher_xml_profile">   |
   |    subscriber_attr.topic.resourceLimitsQos.max_samples = 200; |          <topic>                                         |
   |                                                               |             <resourceLimitsQos>                          |
   |                                                               |                <max_samples>200</max_samples>            |
   |                                                               |             </resourceLimitsQos>                         |
   |                                                               |          </topic>                                        |
   |                                                               |       </publisher>                                       |
   |                                                               |                                                          |
   |                                                               |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                                               |          <topic>                                         |
   |                                                               |             <resourceLimitsQos>                          |
   |                                                               |                <max_samples>200</max_samples>            |
   |                                                               |             </resourceLimitsQos>                         |
   |                                                               |          </topic>                                        |
   |                                                               |       </subscriber>                                      |
   |                                                               |    </profiles>                                           |
   +---------------------------------------------------------------+----------------------------------------------------------+

Unicast locators
****************

They are network endpoints where the entity will receive data.
For more information about network, see :ref:`network-configuration`.
Publishers and subscribers inherit unicast locators from the participant.
You can set a different set of locators through this attribute.

   +---------------------------------------------------------------+----------------------------------------------------------+
   | C++                                                           | XML                                                      |
   +===============================================================+==========================================================+
   | .. code-block:: c++                                           | .. code-block:: xml                                      |
   |                                                               |                                                          |
   |    Locator_t new_locator;                                     |    <profiles>                                            |
   |    new_locator.port = 7800;                                   |       <publisher profile_name="publisher_xml_profile">   |
   |                                                               |          <unicastLocatorList>                            |
   |    subscriber_attr.unicastLocatorList.push_back(new_locator); |             <locator>                                    |
   |                                                               |                <port>7800</port>                         |
   |    publisher_attr.unicastLocatorList.push_back(new_locator);  |             </locator>                                   |
   |                                                               |          </unicastLocatorList>                           |
   |                                                               |       </publisher>                                       |
   |                                                               |                                                          |
   |                                                               |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                                               |          <unicastLocatorList>                            |
   |                                                               |             <locator>                                    |
   |                                                               |                <port>7800</port>                         |
   |                                                               |             </locator>                                   |
   |                                                               |          </unicastLocatorList>                           |
   |                                                               |       </subscriber>                                      |
   |                                                               |    </profiles>                                           |
   +---------------------------------------------------------------+----------------------------------------------------------+

.. _multicast-locators:

Multicast locators
******************

They are network endpoints where the entity will receive data.
For more information about network, see :ref:`network-configuration`.
By default publishers and subscribers don't use any multicast locator.
This attribute is useful when you have a lot of entities and you want to reduce the network usage.

   +-----------------------------------------------------------------+----------------------------------------------------------+
   | C++                                                             | XML                                                      |
   +=================================================================+==========================================================+
   | .. code-block:: c++                                             | .. code-block:: xml                                      |
   |                                                                 |                                                          |
   |    Locator_t new_locator;                                       |    <profiles>                                            |
   |                                                                 |       <publisher profile_name="publisher_xml_profile">   |
   |    new_locator.set_IP4_address("239.255.0.4");                  |          <multicastLocatorList>                          |
   |    new_locator.port = 7900;                                     |             <locator>                                    |
   |                                                                 |                <address>239.255.0.4</address>            |
   |    subscriber_attr.multicastLocatorList.push_back(new_locator); |                <port>7900</port>                         |
   |                                                                 |             </locator>                                   |
   |    publisher_attr.multicastLocatorList.push_back(new_locator);  |          </multicastLocatorList>                         |
   |                                                                 |       </publisher>                                       |
   |                                                                 |                                                          |
   |                                                                 |       <subscriber profile_name="subscriber_xml_profile"> |
   |                                                                 |          <multicastLocatorList>                          |
   |                                                                 |             <locator>                                    |
   |                                                                 |                <address>239.255.0.4</address>            |
   |                                                                 |                <port>7900</port>                         |
   |                                                                 |             </locator>                                   |
   |                                                                 |          </multicastLocatorList>                         |
   |                                                                 |       </subscriber>                                      |
   |                                                                 |    </profiles>                                           |
   +-----------------------------------------------------------------+----------------------------------------------------------+

Advanced configuration
^^^^^^^^^^^^^^^^^^^^^^

.. _network-configuration:

Setting up network configuration
********************************

*eProsima Fast RTPS* implements an architecture of pluggable network transports. Current version implements two network
transports: UDPv4 and UDPv6. By default, when a :class:`Participant` is created, one built-in UDPv4 network transport is
configured.

You can add custom transport using the attribute ``rtps.userTransports``.

.. code-block:: c++

    //Creation of the participant
    eprosima::fastrtps::ParticipantAttributes part_attr;

    auto customTransport = std::make_shared<UDPv4TransportDescriptor>();
        customTransport->sendBufferSize = 9216;
        customTransport->receiveBufferSize = 9216;

    part_attr.rtps.userTransports.push_back(customTransport);

Also you can disable built-in UDPv4 network transport using the attribute ``rtps.useBuiltinTransports``.

.. code-block:: c++

    eprosima::fastrtps::ParticipantAttributes part_attr;

    part_attr.rtps.useBuiltinTransports = false;

Network endpoints are defined by *eProsima Fast RTPS* as locators. Locators in *eProsima Fast RTPS* are enclosed as type :class:`Locator_t`, which has the following fields:

* ``kind``: Defines the protocol. *eProsima Fast RTPS* currently supports UDPv4 or UDPv6
* ``port``: Port as an UDP/IP port.
* ``address``: Maps to IP address

Listening locators
""""""""""""""""""

*eProsima Fast RTPS* divides listening locators in four categories:

* Metatraffic Multicast Locators: these locators are used to receive metatraffic information using multicast.
  They usually are used by built-in endpoints, like the discovery built-in endpoints. You can set your own locators
  using attribute ``rtps.builtin.metatrafficMulticastLocatorList``.

  .. code-block:: c++

    eprosima::fastrtps::ParticipantAttributes part_attr;

    // This locator will open a socket to listen network messages on UDPv4 port 22222 over multicast address 239.255.0.1
    eprosima::fastrtps::rtps::Locator_t locator.set_IP4_address(239, 255, 0 , 1);
    locator.port = 22222;

    part_attr.rtps.builtin.metatrafficMulticastLocatorList.push_back(locator);

* Metatraffic Unicast Locators: these locators are used to receive metatraffic information using unicast.
  They usually are used by built-in endpoints, like the discovery built-in endpoints. You can set your own locators using
  attribute ``rtps.builtin.metatrafficUnicastLocatorList``.

  .. code-block:: c++

    eprosima::fastrtps::ParticipantAttributes part_attr;

    // This locator will open a socket to listen network messages on UDPv4 port 22223 over network interface 192.168.0.1
    eprosima::fastrtps::rtps::Locator_t locator.set_IP4_address(192, 168, 0 , 1);
    locator.port = 22223;

    part_attr.rtps.builtin.metatrafficUniicastLocatorList.push_back(locator);

* User Multicast Locators: these locators are used to receive user information using multicast. They are used by user
  endpoints. You can set your own locators using attribute ``rtps.defaultMulticastLocatorList``.

  .. code-block:: c++

    eprosima::fastrtps::ParticipantAttributes part_attr;

    // This locator will open a socket to listen network messages on UDPv4 port 22224 over multicast address 239.255.0.1
    eprosima::fastrtps::rtps::Locator_t locator.set_IP4_address(239, 255, 0 , 1);
    locator.port = 22224;

    part_attr.rtps.defaultMulticastLocatorList.push_back(locator);

* User Unicast Locators: these locators are used to receive user information using unicast. They are used by user
  endpoints. You can set your own locators using attributes ``rtps.defaultUnicastLocatorList``.

  .. code-block:: c++

    eprosima::fastrtps::ParticipantAttributes part_attr;

    // This locator will open a socket to listen network messages on UDPv4 port 22225 over network interface 192.168.0.1
    eprosima::fastrtps::rtps::Locator_t locator.set_IP4_address(192, 168, 0 , 1);
    locator.port = 22225;

    part_attr.rtps.defaultUnicastLocatorList.push_back(locator);

By default *eProsima Fast RTPS* calculates the listening locators for the built-in UDPv4 network transport using
well-known ports. These well-known ports are calculated using the following predefined rules:

.. list-table:: Ports used
   :header-rows: 1

   * - Traffic type
     - Well-known port expression
   * - Metatraffic multicast
     - PB + DG * *domainId* + offsetd0
   * - Metatraffic unicast
     - PB + DG * *domainId* + offsetd1 + PG * *participantId*
   * - User multicast
     - PB + DG * *domainId* + offsetd2
   * - User unicast
     - PB + DG * *domainId* + offsetd3 + PG * *participantId*

These predefined rules use some values explained here:

* DG: DomainId Gain. You can set this value using attribute ``rtps.port.domainIDGain``. Default value is ``250``.
* PG: ParticipantId Gain. You can set this value using attribute ``rtps.port.participantIDGain``. Default value is ``2``.
* PB: Port Base number. You can set this value using attribute ``rtps.port.portBase``. Default value is ``7400``.
* offsetd0, offsetd1, offsetd2, offsetd3: Additional offsets. You can set these values using attributes
  ``rtps.port.offsetdN``. Default values are: ``offsetd0 = 0``, ``offsetd1 = 10``, ``offsetd2 = 1``, ``offsetd3 = 11``.

A UDPv4 unicast locator supports to have a null address. In that case *eProsima Fast RTPS* understands to get local network
addresses and use them.

A UDPv4 locator support to have a zero port. In that case *eProsima Fast RTPS* understands to calculate well-known port
for that type of traffic.

Sending locators
""""""""""""""""

These locators are used to create network endpoints to send all network messages. You can set your own locators using
the attribute ``rtps.defaultOutLocatorList``.

.. code-block:: c++

   eprosima::fastrtps::ParticipantAttributes part_attr;

   // This locator will create a socket to send network message on UDPv4 port 34000 over network interface 192.168.0.1
   Locator_t locator.set_IP4_address(192.168.0.1);
   locator.port = 34000;

   part_attr.rtps.defaultOutLocatorList.push_back(locator);

By default *eProsima Fast RTPS* sends network messages using a random UDPv4 port over all interface networks.

A UDPv4 unicast locator supports to have a null address. In that case *eProsima Fast RTPS* understands to get local network
addresses and use them.

A UDPv4 locator support to have a zero port. In that case *eProsima Fast RTPS* understands to get a random UDPv4 port.

.. _initial-peers:

Initial peers
"""""""""""""

These locators are used to know where to send initial discovery network messages. You can set your own locators using
attribute ``rtps.builtin.initialPeersList``. By default *eProsima Fast RTPS* uses as initial peers the Metatraffic
Multicast Locators.

.. code-block:: c++

   eprosima::fastrtps::ParticipantAttributes part_attr;

   // This locator configures as initial peer the UDPv4 address 192.168.0.2:7600.
   // Initial discovery network messages will send to this UDPv4 address.
   Locator_t locator.set_IP4_address(192.168.0.2);
   locator.port = 7600;

   part_attr.rtps.builtin.initialPeersList.push_back(locator);

Tips
""""

**Disabling all multicast traffic**

.. code-block:: c++

   eprosima::fastrtps::ParticipantAttributes part_attr;

   // Metatraffic Multicast Locator List will be empty.
   // Metatraffic Unicast Locator List will contain one locator, with null address and null port.
   // Then eProsima Fast RTPS will use all network interfaces to receive network messages using a well-known port.
   Locator_t default_unicast_locator;
   participant_attr_.rtps.builtin.metatrafficUnicastLocatorList.push_back(default_unicast_locator);

   // Initial peer will be UDPv4 addresss 192.168.0.1. The port will be a well-known port.
   // Initial discovery network messages will be sent to this UDPv4 address.
   Locator_t initial_peer;
   initial_peer.set_IP4_address(192, 168, 0, 1);
   participant_attr_.rtps.builtin.initialPeersList.push_back(initial_peer);

.. _xml-profiles:

XML profiles
------------

In the :ref:`configuration` section you could see how to configure entity attributes using XML profiles, but this section goes
deeper into it.

XML profiles are loaded from XML files. *eProsima Fast RTPS* permits to load as much XML files as you want. An XML file
can contain several XML profiles. An XML profile is defined by a unique name that is used to reference the XML profile
when you create a Fast RTPS entity. *eProsima Fast RTPS* will also try to find in current execution path and load an XML file with the name
*DEFAULT_FASTRTPS_PROFILES.xml*. If this file exists, it is loaded at the library initialization.

Making an XML
^^^^^^^^^^^^^

An XML file can contain several XML profiles. They can be divided in participant, publisher and
subscriber profiles.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles>
        <participant profile_name="participant_profile">
            ....
        </participant>

        <publisher profile_name="publisher_profile">
            ....
        </publisher>

        <subscriber profile_name="subscriber_profile">
            ....
        </subscriber>
    </profiles>

The entire list of supported attributes can be checked in this `XSD file <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_.

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before creating any entity, you can load XML files using ``Domain::loadXMLProfilesFile`` function.
``createParticipant``, ``createPublisher`` and ``createSubscriber`` have a version that expects the profile name as
argument. *eProsima Fast RTPS* searches the XML profile using this profile name and applies the XML profile to the
entity.

.. code-block:: c++

   eprosima::fastrtps::Domain::loadXMLProfilesFile("my_profiles.xml");

   Participant *participant = Domain::createParticipant("participant_xml_profile");
   Publisher *publisher = Domain::createPublisher(participant, "publisher_xml_profile");
   Subscriber *subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");


Additional Concepts
-------------------

Using message meta-data
^^^^^^^^^^^^^^^^^^^^^^^

When a message is taken from the Subscriber, an auxiliary :class:`SampleInfo_t` structure instance is also returned.

.. code-block:: c++

    HelloWorld m_Hello;
    SampleInfo_t m_info;
    sub->takeNextData((void*)&m_Hello, &m_info);

This :class:`SampleInfo_t` structure contains meta-data on the incoming message:

* sampleKind: type of the sample, as defined by the RTPS Standard. Healthy messages from a topic are always ALIVE.
* WriterGUID: Signature of the sender (Publisher) the message comes from.
* OwnershipStrength: When several senders are writing the same data, this field can be used to determine which data is more reliable.
* SourceTimestamp: A timestamp on the sender side that indicates the moment the sample was encapsulated and sent.

This meta-data can be used to implement filters:

.. code-block:: c++

    if((m_info->sampleKind == ALIVE)& (m_info->OwnershipStrength > 25 ){
        //Process data
   }

Defining callbacks
^^^^^^^^^^^^^^^^^^

As we saw in the example, both the :class:`Publisher` and :class:`Subscriber` have a set of callbacks you can use
in your application. These callbacks are to be implemented within classes that derive from
:class:`SubscriberListener` or :class:`PublisherListener`. The following table gathers information about
the possible callbacks that can be implemented in both cases:

        +-----------------------+-----------+------------+
        |      Callback         | Publisher | Subscriber |
        +=======================+===========+============+
        |   onNewDataMessage    |     N     |      Y     |
        +-----------------------+-----------+------------+
        | onSubscriptionMatched |     N     |      Y     |
        +-----------------------+-----------+------------+
        | onPublicationMatched  |     Y     |      N     |
        +-----------------------+-----------+------------+
