.. _getting_started_brief_intro:

A brief introduction to the RTPS protocol
-----------------------------------------

At the top of RTPS, we find the Domain, which defines a separate plane of communication.
Several domains can coexist at the same time independently.
A domain contains any number of Participants, elements capable of sending and receiving data.
To do this, the participants use their Endpoints:

* Reader: Endpoint able to receive data.
* Writer: Endpoint able to send data.

A Participant can have any number of writer and reader endpoints.

.. image:: /01-figures/RTPS-structure.png

Communication revolves around Topics, which define the data being exchanged.
Topics don’t belong to any participant in particular; instead, all interested participants keep track of changes to the
topic data and make sure to keep each other up to date.
The unit of communication is called a Change, which represents an update to a topic.
Endpoints register these changes on their History, a data structure that serves as a cache for recent changes.
When you publish a change through a writer endpoint, the following steps happen behind the scenes:

* The change is added to the writer’s history cache.
* The writer informs any readers it knows about.
* Any interested (subscribed) readers request the change.
* After receiving data, readers update their history cache with the new change.

By choosing Quality of Service policies, you can affect how these history caches are managed in several ways, but the
communication loop remains the same. You can read more information in :ref:`configuration`.


