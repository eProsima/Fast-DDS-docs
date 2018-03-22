Library Overview
================


You can interact with Fast RTPS at two different levels:

* Publisher-Subscriber: Simplified abstraction over RTPS.
* Writer-Reader: Direct control over RTPS endpoints.

.. image:: architecture.png

In red, the Publisher-Subscriber layer offers a convenient abstraction for most use cases. It allows you to define Publishers and Subscribers associated to a topic, and a simple way to transmit topic data. You may remember this from the example we generated in the "Getting Started" section, where we updated our local copy of the topic data, and called a write() method on it.
In blue, the Writer-Reader layer is closer to the concepts defined in the RTPS standard, and allows a finer control, but requires you to interact directly with history caches for each endpoint.

Fast RTPS architecture
----------------------

Threads
^^^^^^^

eProsima Fast RTPS is concurrent and event-based. Each participant spawns a set of threads to take care of background tasks such as logging, message reception and asynchronous communication.
This should not impact the way you use the library: the public API is thread safe, so you can fearlessly call any methods on the same participant from different threads. However, it is still useful to know how Fast RTPS schedules work:

* Main thread: Managed by the application.
* Event thread: Each participant owns one of these, and it processes periodic and triggered events.
* Asynchronous writer thread: This thread manages asynchronous writes for all participants. Even for synchronous writers, some forms of communication must be initiated in the background.
* Reception threads: Participants spawn a thread for each reception channel, where the concept of channel depends on the transport layer (e.g. an UDP port).

Events
^^^^^^

There is an event system that enables Fast RTPS to respond to certain conditions, as well as schedule periodic activities. Few of them are visible to the user, since most are related to RTPS metadata. However, you can define your own periodic events by inheriting from the TimedEvent class.
