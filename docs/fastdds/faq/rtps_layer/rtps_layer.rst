.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_rtps_layer_questions:

RTPS LAYER Frequently Asked Questions
=====================================

.. collapse::  What is the primary function of the RTPS Layer in *eprosima Fast DDS*, and how does it differ from the DDS Layer?

    |br|

    The lower level RTPS Layer of eprosima Fast DDS serves as an implementation of the protocol defined in the RTPS standard. This layer provides more control over the internals of the communication protocol than the DDS Layer, so advanced users have finer control over the library's functionalities. For further information, see :ref:`rtps_layer`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary responsibility of an "RTPSParticipant" in the context of the RTPS Layer?

    |br|

    As the RTPS standard specifies, ``RTPSWriters`` and ``RTPSReaders`` are always associated with a ``History`` element. In the DDS Layer, its creation and management is hidden, but in the RTPS Layer, you have full control over its creation and configuration. For further information, see :ref:`rtps_layer`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of creating a "WriterHistory" when creating an RTPS Writer?

    |br|

    Writers are created with |RTPSDomain::createRTPSWriter-api| and configured with a ``WriterAttributes`` structure. They also need a ``WriterHistory`` which is configured with a ``HistoryAttributes`` structure. For further information, see :ref:`rtps_layer`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of using the "History" element in the RTPS Layer?

    |br|

    In the RTPS Protocol, Readers and Writers save the data about a topic in their associated Histories. Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as ``CacheChange_t``. Changes are always managed by the History. For further information, see :ref:`rtps_layer`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How can a custom Payload Pool improve the performance of Writers and Readers in RTPS, and what should be considered when implementing one?

    |br|

    A custom payload pool can improve the performance of writers and readers in RTPS by optimizing memory usage and reducing costly memory allocation operations, especially when dealing with large or variable-sized data. When implementing one, it is essential to ensure that the payload size accommodates the serialized user data plus metadata, and to choose a strategy (e.g., preallocated, dynamic) that balances memory usage and allocation efficiency based on application needs. For further information, see :ref:`rtps_layer_custom_payload_pool`.

|
