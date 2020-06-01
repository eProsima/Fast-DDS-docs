Advanced Functionalities
########################


This section covers slightly more advanced, but useful features that enrich your implementation.


.. _topics-and-keys:

Topics and Keys
***************

The RTPS standard contemplates the use of keys to define multiple data sources/sinks within a single topic.

There are three ways of implementing keys into your topic:

* Defining a `@Key` field in the IDL file when using Fast DDS-Gen (see the examples that come with the distribution).
* Manually implementing and using a :func:`getKey()` method.
* Adding the attribute `Key` to the member and its parents when using dynamic types (see :ref:`dynamic-types`).

Publishers and Subscribers using topics with keys must be configured to use them, otherwise, they will have no effect:

+-------------------------------------------------+
| **C++**                                         |
+-------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp     |
|    :language: c++                               |
|    :start-after: //CONF-QOS-KEY                 |
|    :end-before: //!--                           |
+-------------------------------------------------+
| **XML**                                         |
+-------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml      |
|    :language: xml                               |
|    :start-after: <!-->CONF-QOS-KEY              |
|    :end-before: <!--><-->                       |
+-------------------------------------------------+

The RTPS Layer requires you to call the :func:`getKey()` method manually within your callbacks.

You can tweak the History to accommodate data from multiple keys based on your current configuration.
This consist of defining a maximum number of data sinks and a maximum size for each sink:

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |
|    :start-after: //CONF-QOS-RESOURCELIMIT-INSTANCES    |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-QOS-RESOURCELIMIT-INSTANCES |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

Note that your History must be big enough to accommodate the maximum number of samples for each key.
eProsima Fast DDS will notify you if your History is too small.




Subscribing to Discovery Topics
*******************************

As specified in the :ref:`discovery` section, the Participant or RTPS Participant has a series of meta-data endpoints
for use during the discovery process.
The participant listener interface includes methods which are called each time a Publisher or a Subscriber is
discovered.
This allows you to create your own network analysis tools.

+--------------------------------------------------+
| **Implementation of custom listener**            |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-LISTENER |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **Setting the custom listener**                  |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-SET      |
|    :end-before: //!--                            |
+--------------------------------------------------+

The callbacks defined in the ReaderListener you attach to the EDP will execute for each data message after
the built-in protocols have processed it.





