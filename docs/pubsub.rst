Publisher-Subscriber Layer
==========================

*eProsima Fast RTPS* provides a high-level Publisher-Subscriber Layer, which is a simple to use abstraction over the
RTPS protocol.
By using this layer, you can code a straight-to-the-point application while letting the library take care of the lower
level configuration.

How to use the Publisher-Subscriber Layer
-----------------------------------------

We are going to use the example built in the previous section to explain how this layer works.

The first step is to create a :class:`Participant` instance, which will act as a container for the Publishers and
Subscribers our application needs. For this we use :class:`Domain`, a static class that manages RTPS entities.
We also need to pass a configuration structure for the Participant, which can be left in its default configuration for
now:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CREATE_PARTICIPANT
    :end-before: //!--

The default configuration provides a basic working set of options with predefined ports for communications.
During this tutorial, you will learn to tune *eProsima Fast RTPS*.

In order to use our topic, we have to register it within the :class:`Participant` using the code generated with
*fastrtpsgen* (see :ref:`fastrtpsgen-intro`.
Once again, this is done by using the :class:`Domain` class:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_REGISTER_TYPE
    :end-before: //!--

Once set up, we instantiate a :class:`Publisher` within our :class:`Participant`:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CREATE_PUBLISHER
    :end-before: //!--

Once the :class:`Publisher` is functional, posting data is a simple process:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_WRITE_SAMPLE
    :end-before: //!--

The :class:`Publisher` has a set of optional callback functions that are triggered when events happen.
An example is when a :class:`Subscriber` starts listening to our topic.

To implement these callbacks we create the class :class:`PubListener`, which inherits from the base class
:class:`PublisherListener`.
We pass an instance to this class during the creation of the :class:`Publisher`.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_PUBLISHER_LISTENER
    :end-before: //!--

The :class:`Subscriber` creation and implementation are symmetric.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CREATE_SUBSCRIBER
    :end-before: //!--

Incoming messages are processed within the callback that is called when a new message is received:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_SUBSCRIBER_LISTENER
    :end-before: //!--

.. _configuration:

Configuration
-------------

*eProsima Fast RTPS* entities can be configured through the code or XML profiles.
This section will show both alternatives.

.. _participantconfiguration:

Participant configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`Participant` can be configured via the :class:`ParticipantAttributes` structure.
``createParticipant`` function accepts an instance of this structure.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CONF_PARTICIPANT
    :end-before: //!--

Also, it can be configured through an XML profile.
``createParticipant`` function accepts a name of an XML profile.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CONF_PARTICIPANT_XML
    :end-before: //!--

About XML profiles you can learn more in :ref:`xml-profiles`.
This is an example of a participant XML profile.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PARTICIPANT
    :end-before: <!--><-->

We will now go over the most common configuration options.

* **Participant name:** the name of the :class:`Participant` forms part of the meta-data of the RTPS protocol.

  +--------------------------------------------------------+
  | **C++**                                                |
  +--------------------------------------------------------+
  | .. literalinclude:: ../code/CodeTester.cpp             |
  |    :language: c++                                      |
  |    :start-after: //PUBSUB_API_CONF_PARTICIPANT_NAME    |
  |    :end-before: //!--                                  |
  +--------------------------------------------------------+
  | **XML**                                                |
  +--------------------------------------------------------+
  | .. literalinclude:: ../code/XMLTester.xml              |
  |    :language: xml                                      |
  |    :start-after: <!-->PUBSUB_API_CONF_PARTICIPANT_NAME |
  |    :end-before: <!--><-->                              |
  +--------------------------------------------------------+

* **DomainId:** Publishers and Subscribers can only talk to each other if their Participants belong to the same
  DomainId.

  +----------------------------------------------------------+
  | **C++**                                                  |
  +----------------------------------------------------------+
  | .. literalinclude:: ../code/CodeTester.cpp               |
  |    :language: c++                                        |
  |    :start-after: //PUBSUB_API_CONF_PARTICIPANT_DOMAIN    |
  |    :end-before: //!--                                    |
  +----------------------------------------------------------+
  | **XML**                                                  |
  +----------------------------------------------------------+
  | .. literalinclude:: ../code/XMLTester.xml                |
  |    :language: xml                                        |
  |    :start-after: <!-->PUBSUB_API_CONF_PARTICIPANT_DOMAIN |
  |    :end-before: <!--><-->                                |
  +----------------------------------------------------------+

.. _pubsubconfiguration:

Publisher and Subscriber configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`Publisher` can be configured via the :class:`PublisherAttributes` structure and
``createPublisher`` function accepts an instance of this structure. The :class:`Subscriber` can be configured via the
:class:`SubscriberAttributes` structure and ``createSubscriber`` function accepts an instance of this structure.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CONF_CREATE_PUBSUB
    :end-before: //!--

Also, these entities can be configured through an XML profile. ``createPublisher`` and ``createSubscriber`` functions
accept the name of an XML profile.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //PUBSUB_API_CONF_CREATE_PUBSUB_XML
    :end-before: //!--

We will now go over the most common configuration options.

.. _Topic_information:

Topic information
*****************

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange
messages.

+----------------------------------------------------+
| **C++**                                            |
+----------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp         |
|    :language: c++                                  |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_TOPIC    |
|    :end-before: //!--                              |
+----------------------------------------------------+
| **XML**                                            |
+----------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml          |
|    :language: xml                                  |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_TOPIC |
|    :end-before: <!--><-->                          |
+----------------------------------------------------+

.. _reliability:

Reliability
***********

The RTPS standard defines two behavior modes for message delivery:

   * Best-Effort (default): Messages are sent without arrival confirmation from the receiver (subscriber).
     It is fast, but messages can be lost.

   * Reliable: The sender agent (publisher) expects arrival confirmation from the receiver (subscriber).
     It is slower but prevents data loss.

+----------------------------------------------------------+
| **C++**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp               |
|    :language: c++                                        |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_RELIABILITY    |
|    :end-before: //!--                                    |
+----------------------------------------------------------+
| **XML**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml                |
|    :language: xml                                        |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_RELIABILITY |
|    :end-before: <!--><-->                                |
+----------------------------------------------------------+

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

+------------------------------------------------------+
| **C++**                                              |
+------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp           |
|    :language: c++                                    |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_HISTORY    |
|    :end-before: //!--                                |
+------------------------------------------------------+
| **XML**                                              |
+------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml            |
|    :language: xml                                    |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_HISTORY |
|    :end-before: <!--><-->                            |
+------------------------------------------------------+

.. _durability-qos:

Durability
**********

Durability configuration of the endpoint defines how it behaves regarding samples that existed on the topic before a
subscriber joins

   * Volatile: Past samples are ignored, a joining subscriber receives samples generated after the moment it matches.
   * Transient Local (Default): When a new subscriber joins, its History is filled with past samples.
   * Transient: When a new subscriber joins, its History is filled with past samples, which are stored on persistent
     storage (see :ref:`persistence`).

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp              |
|    :language: c++                                       |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_DURABILITY    |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml               |
|    :language: xml                                       |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_DURABILITY |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

.. _resourceLimits-qos:

Resource limits
***************

Allow controlling the maximum size of the History and other resources.

+--------------------------------------------------------------+
| **C++**                                                      |
+--------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                   |
|    :language: c++                                            |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_RESOURCE_LIMITS    |
|    :end-before: //!--                                        |
+--------------------------------------------------------------+
| **XML**                                                      |
+--------------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml                    |
|    :language: xml                                            |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_RESOURCE_LIMITS |
|    :end-before: <!--><-->                                    |
+--------------------------------------------------------------+

Unicast locators
****************

They are network endpoints where the entity will receive data.
For more information about the network, see :ref:`comm-transports-configuration`.
Publishers and subscribers inherit unicast locators from the participant.
You can set a different set of locators through this attribute.

+---------------------------------------------------------------+
| **C++**                                                       |
+---------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                    |
|    :language: c++                                             |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_UNICAST_LOCATORS    |
|    :end-before: //!--                                         |
+---------------------------------------------------------------+
| **XML**                                                       |
+---------------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml                     |
|    :language: xml                                             |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_UNICAST_LOCATORS |
|    :end-before: <!--><-->                                     |
+---------------------------------------------------------------+

.. _multicast-locators:

Multicast locators
******************

They are network endpoints where the entity will receive data.
For more information about network configuration, see :ref:`comm-transports-configuration`.
By default publishers and subscribers don't use any multicast locator.
This attribute is useful when you have a lot of entities and you want to reduce the network usage.

+-----------------------------------------------------------------+
| **C++**                                                         |
+-----------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                      |
|    :language: c++                                               |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_MULTICAST_LOCATORS    |
|    :end-before: //!--                                           |
+-----------------------------------------------------------------+
| **XML**                                                         |
+-----------------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml                       |
|    :language: xml                                               |
|    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_MULTICAST_LOCATORS |
|    :end-before: <!--><-->                                       |
+-----------------------------------------------------------------+

Additional Concepts
-------------------

Using message meta-data
^^^^^^^^^^^^^^^^^^^^^^^

When a message is taken from the Subscriber, an auxiliary :class:`SampleInfo_t` structure instance is also returned.

+--------------------------------------------------------------+
| **Static types**                                             |
+--------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                   |
|    :language: c++                                            |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_STATIC_SAMPLEINFO  |
|    :end-before: //!--                                        |
+--------------------------------------------------------------+
| **Dynamic types**                                            |
+--------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                   |
|    :language: c++                                            |
|    :start-after: //PUBSUB_API_CONF_PUBSUB_DYNAMIC_SAMPLEINFO |
|    :end-before: //!--                                        |
+--------------------------------------------------------------+

This :class:`SampleInfo_t` structure contains meta-data on the incoming message:

* `sampleKind`: type of the sample, as defined by the RTPS Standard.
  Healthy messages from a topic are always ALIVE.
* `WriterGUID`: Signature of the sender (Publisher) the message comes from.
* `OwnershipStrength`: When several senders are writing the same data, this field can be used to determine which data is
  more reliable.
* `SourceTimestamp`: A timestamp on the sender side that indicates the moment the sample was encapsulated and sent.

This meta-data can be used to implement filters:

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //PUBSUB_API_CONF_PUBSUB_SAMPLEINFO_USAGE
   :end-before: //!--

Defining callbacks
^^^^^^^^^^^^^^^^^^

As we saw in the example, both the :class:`Publisher` and :class:`Subscriber` have a set of callbacks you can use
in your application. These callbacks are to be implemented within classes that derive from
:class:`SubscriberListener` or :class:`PublisherListener`. The following table gathers information about
the possible callbacks that can be implemented in both cases:

+-------------------------+-----------+------------+
|      Callback           | Publisher | Subscriber |
+=========================+===========+============+
|   `onNewDataMessage`    |     N     |      Y     |
+-------------------------+-----------+------------+
| `onSubscriptionMatched` |     N     |      Y     |
+-------------------------+-----------+------------+
| `onPublicationMatched`  |     Y     |      N     |
+-------------------------+-----------+------------+
