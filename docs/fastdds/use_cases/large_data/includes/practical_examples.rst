Practical Examples
------------------

.. _use-case-largeData-exampleFile:

Example: Sending a large file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Consider the following scenario:

* A Publisher needs to send a file with a size of 9.9 MB.
* The Publisher and Subscriber are connected through a network
  with a bandwidth of 100 MB/s

With a fragment size of 64 kB, the Publisher has to send about 1100 fragments to send the whole file.
A possible configuration for this scenario could be:

* Using |RELIABLE_RELIABILITY_QOS-api|,
  since a losing a single fragment would mean the loss of the complete file.
* Decreasing the heartbeat period, in order to increase the reactivity of the Publisher.
* Limiting the data rate using a :ref:`Flow Controller<flow-controllers>`,
  to avoid this transmission cannibalizing the whole bandwidth.
  A reasonable rate for this application could be 5 MB/s, which represents only 5% of the total bandwidth.

.. note::

   Using :ref:`transport_sharedMemory_sharedMemory` the only limit to the fragment size is the available memory.
   Therefore, all fragmentation can be avoided in SHM by increasing the size of the shared buffers.

.. _use-case-largeData-exampleStreaming:

Example: Video streaming
^^^^^^^^^^^^^^^^^^^^^^^^

In this scenario, the application transmits a video stream between a Publisher
and a Subscriber, at 50 fps. In real-time audio or video transmissions,
it is usually preferred to have a high stable datarate feed, even at the cost of losing some
samples.
Losing one or two samples per second at 50 fps is more acceptable than freezing the video waiting for the retransmission
of lost samples.
Therefore, in this case |BEST_EFFORT_RELIABILITY_QOS-api| can be appropriate.
