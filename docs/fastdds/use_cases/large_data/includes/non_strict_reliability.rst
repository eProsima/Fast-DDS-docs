.. _tuning-nonstrict-reliability:

Using Non-strict Reliability
----------------------------

When :ref:`historyqospolicykind` is set as |KEEP_ALL_HISTORY_QOS-api|, all samples have to be received
(and acknowledged) by all subscribers before they can be overridden by the DataWriter.
If the message rate is high and the network is not reliable (i.e., lots of packets get lost), the history of the
DataWriter can be filled up, blocking the publication of new messages until any
of the old messages is acknowledged by all subscribers.

If this strictness is not needed, :ref:`historyqospolicykind` can be set as |KEEP_LAST_HISTORY_QOS-api|.
In this case, when the history of the DataWriter is full, the oldest message that has not
been fully acknowledged yet is overridden with the new one.
If any subscriber did not receive the discarded message, the publisher
will send a GAP message to inform the subscriber that the message is lost forever.
