Advanced Functionalities
========================


This section covers slightly more advanced, but useful features that enrichen your implementation.


Topics and Keys
---------------

The RTPS standard contemplates the use of keys to define multiple data sources/sinks within a single topic.

There are two ways of implementing keys into your topic:

* Defining a `@Key` field in the IDL file when using FastRTPSGen (see the examples that come with the distribution).
* Manually implementing and using a getKey() method.

Publishers and Subscribers using topics with keys must be configured to use them, otherwise they will have no effect: ::

        //Publicher-Subscriber Layer configuration
        PubAttributes.topic.topicKind = WITH_KEY

	
The RTPS Layer requires you to call the getKey() method manually within your callbacks.

Sending large data
------------------

The default size *eProsima Fast RTPS* uses to create sockets is a conservative value of 65kb. If your topic data is bigger, it must be fragmented.

Fragmented messages are sent over multiple packets, as understood by the particular transport layer. To make this possible, you must configure the Publisher to work in asynchronous mode. ::

        PublisherAttributes Wparam;
        Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;	// Allows fragmentation

In the Writer-Subscriber layer, you have to configure the Writer: ::

        WriterAttributes Wparam;
        Wparam.mode= ASYNCHRONOUS_WRITER;	// Allows fragmentation
	
Note that in best-effort mode messages can be lost if you send big data too fast and the buffer is filled at a faster rate than what the client can process messages.

Tuning Realiable mode
---------------------

RTPS uses Heartbeat messages to make matched endpoints exchange meta-data on what pieces of data they hold so the missing ones can be re-sent (on Reliable mode of course).
You can modify the frequency of this meta-data exchange by specifying a custom heartbeat period.

The heartbeat period in the Publisher-Subscriber level is configured as part of the `ParticipantAttributes`: ::

        PublisherAttributes pubAttr;
        pubAttr.times.heartbeatPeriod.seconds = 0;
        pubAttr.times.heartbeatPeriod.fraction = 500;	//500 ms
	
In the Writer-Reader layer, this belong to the `WriterAttributes`: ::

        WriterAttributes Wattr;
        Wattr.times.heartbeatPeriod.seconds = 0;
        Wattr.times.heartbeatPeriod.fraction = 500;
	
A smaller heartbeat period increases the amount of overhead messages in the network, but speeds up the system response when a piece of data
is lost.

Flow Controllers
----------------

*eprosima Fast RTPS* supports user configurable flow controllers on a Publisher and Participant level. These
controllers can be used to limit the amount of data to be sent under certain conditions depending on the
kind of controller implemented.

The current release implement throughput controllers, which can be used to limit the total message throughput to be sent
over the network per time measurement unit. In order to use them, a descriptor must be passed into the Participant or Publisher Attributes. ::

        PublisherAttributes WparamSlow;
        ThroughputControllerDescriptor slowPublisherThroughputController{300000, 1000};	//Limit to 300kb per second
        WparamSlow.terminalThroughputController = slowPublisherThroughputController;

In the Writer-Reader layer, the throughput controllers is built-in and the descriptor deafaults to infinite throughput. To change the values: ::

        WriterAttributes WParams;
        WParams.throughputController.size = 300000;	//300kb
        WParams.throughputController.timeMS = 1000;	//1000ms
	
Note that specifying a throughput controller with a size smaller than the socket size can cause messages to never become sent.

Transport Layer
---------------

Unless you specify other configuration, *eprosima Fast RTPS* will use its built in UDPv4 Transport Layer with a default configuration. You can change this default configuration or switch to UDPv6 
by providing an alternative configuration when you create the Participant. ::

        RTPSParticipantAttributes Pparams;
        auto my_transport = std::make_shared<UDPv6Transport::TransportDescriptor>();	//Create a descriptor for the new transport
        my_transport->receiveBufferSize = 65536;	//Configuration parameters
        my_transport->granularMode = false;
        Pparams.useBuiltinTransport = false;		//Disable the built-in Transport Layer.
        Pparams.userTransports.push_back(my_transport);		//Link the Transport Layer to the Participant

Note that unless you manualy disable the built-in transport layer, the Participant will use your custom transport configuration along the built-in one.
	
This distribution comes with an example of how to change the configuration of the transport layer. It is in folder `examples\UserDefinedTransportExample`

Matching endpoints the manual way
---------------------------------

By default, when you create a Participant or a RTPS Participant the built-in protocols for automatic discovery of endpoints will be active. You can disable them by configuring the Participant: ::

        ParticipantAttributes Pparam;
        Pparam.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = false;
        Pparam.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = false;

If you disable the built-in discovery protocols, you will need to manually match Readers and Writers. To inform a Writer about a remote Reader, you can either provide an XML configuration file or use the `RemoteReaderAttributes` structure: ::

        RemoteReaderAttributes ratt;
        Locator_t loc;		//Add the locator that represents a channel the Reader listens to
        loc.set_IP4_address(127,0,0,1);
        loc.port = 22222;
        ratt.endpoint.unicastLocatorList.push_back(loc)
        ratt.guid = c_Guid_Unknown; //GUID_t is left blank, but must be configured when using Reliable Mode.
        writer->matched_writer_add(ratt);

Registering a remote Writer into a Reliable mode Reader works the same way: ::

        RemoteWriterAttributes watt;
        //Configure watt
        reader->matched_reader_add(watt);

If you decide to provide the information via XML, you have to specify the file where you want to load from: ::

        participant_attributes.rtps.builtin.use_STATIC_EndpointDiscoveryProtocol = true;
        participant_attributes.rtps.builtin.setStaticEndpointXMLFilename("my_xml_configuration.xml");

You can use this sample XML as a base for building your configuration files: ::

        <staticdiscovery>
            <participant>
                <name>RTPSParticipant</name>
                <reader>
                    <userId>3</userId>
                    <entityId>4</entityId>
                    <expectsInlineQos>false</expectsInlineQos>
                    <topicName>TEST_TOPIC_NAME</topicName>
                    <topicDataType>HelloWorldType</topicDataType>
                    <topicKind>NO_KEY</topicKind>
                    <reliabilityQos>RELIABLE_RELIABILITY_QOS</reliabilityQos>
                    <unicastLocator
                        address="127.0.0.1"
                        port="31377">    
                    </unicastLocator>
                    <multicastLocator
                        address="127.0.0.1"
                        port="31378">
                    </multicastLocator>
                    <durabilityQos>TRANSIENT_LOCAL_DURABILITY_QOS</durabilityQos>
                </reader>
            </participant>
        </staticdiscovery>

Making the most out of the built-in protocols
-----------------------------------------

As specified in the Built-In protocols section, the Participant or RTPS Participant has a series of meta-data endpoints for use during the discovery process.
It is possible to create a custom listener that listens to the Endpoint Discovery Protocol meta-data. This allows you to create your own network analysis tools. ::

        CustomReaderListener *my_readerListenerSub = new(CustomReaderListener); // Custom user ReaderListeners
        CustomReaderListener *my_readerListenerPub = new(CustomReaderListener); 
        std::pair<StatefulReader*,StatefulReader*> EDPReaders = my_participant->getEDPReaders(); //Get access to the EDP endpoints
        EDPReaders.first()->setListener(my_readerListenerSub);	// Perform attachments
        EDPReaders.second()->setListener(my_readerListenerPub);

The callbacks defined in the ReaderListener you attach to the EDP will execute for each data message after the built-in protocols have processed it.

Additional Quality of Service options
-------------------------------------

As a user, you can implement your own quality of service (QoS) restrictions in your application. *eProsima Fast RTPS* comes bundles with a set of examples of how to implement common client-wise QoS settings:

* Deadline: Rise an alarm when the frequency of message arrival for a topic falls below a certain threshold.
* Ownership Srength: When multiple data sources come online, filter duplicates by focusing on the higher priority sources.
* Filtering: Filter incoming messages based on content, time, or both.

These examples come with their own `Readme.txt` that explains how the implementations work.


This marks the end of this document. We recommend you to take a look at the doxygen API reference and the embedded examples that come with the distribution. If you need more help, send us an email it `support@eprosima.com`.

