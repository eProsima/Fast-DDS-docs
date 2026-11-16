.. _tuning-heartbeat-period:

Tuning Heartbeat Period
-----------------------

On |RELIABLE_RELIABILITY_QOS-api| (:ref:`reliabilityqospolicy`), RTPS protocol can detect which messages have been lost
and retransmit them.
This mechanism is based on meta-traffic information exchanged between
DataWriters and DataReaders,
namely, Heartbeat and Ack/Nack messages.

A smaller Heartbeat period increases the CPU and network overhead, but speeds up the system response when
a piece of data is lost.
Therefore, users can customize the Heartbeat period to match their needs.
This can be done with the DataWriterQos.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_TUNING_RELIABLE_WRITER
   :end-before: //!--
   :dedent: 8
