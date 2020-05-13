Functionalities
---------------

Fast DDS has some added features that can be implemented and configured by the user in their application.
These are outlined below.

Discovery Protocols
^^^^^^^^^^^^^^^^^^^

The discovery protocols define the mechanisms by which DataWriters publishing under a given Topic, and DataReaders
subscribing to that same Topic are matched, so that they can start sharing data.
This applies at any point in the communication process.
Fast DDS provides the following discovery mechanisms:

* **Simple Discovery**.
  This is the default discovery mechanism, which is defined in the
  `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2>`_ and provides compatibility with other DDS implementations
  Here the DomainParticipants are discovered individually at an early stage to subsequently match the DataWriter and
  DataReader they implement.
* **Server-Client Discovery**.
  This discovery mechanism uses a centralized discovery architecture, where servers act as a hubs for discovery meta
  traffic.
* **Static Discovery**.
  This implements the discovery of DomainParticipants to each other but it is possible to skip the discovery of the
  entities contained in each DomainParticipant (DataReader/DataWriter) if these entities are known in advance by the
  remote DomainParticipants.
* **Manual Discovery**.
  This mechanism is only compatible with the RTPS layer.
  It allows the user to manually match and unmatch RTPSParticipants, RTPSWriters, and RTPSReaders using whatever
  external meta-information channel of its choice.

The detailed explanation and configuration of all the discovery protocols implemented in Fast DDS can be seen in
the :ref:`discovery` section.

Security
^^^^^^^^

Fast DDS can be configured to provide secure communications by implementing pluggable security at three levels:

* Authentication of remote DomainParticipants.
  The **DDS:Auth:PKI-DH** plugin provides authentication using a trusted Certificate
  Authority (CA) and ECDSA Digital Signature Algorithms to perform the mutual authentication.
  It also establishes a shared secret using Elliptic Curve Diffie-Hellman (ECDH) Key Agreement protocol.
* Access control of entities.
  The **DDS:Access:Permissions** plugin provides access control to DomainParticipants at the DDS Domain and Topic level.
* Encryption of data.
  The **DDS:Crypto:AES-GCM-GMAC** plugin provides authenticated encryption using Advanced Encryption Standard (AES) in
  Galois Counter Mode (AES-GCM).

More information about security configuration in Fast DDS is available in the :ref:`security` section.

Logging
^^^^^^^

Fast DDS provides an extensible Logging system.
:class:`Log` class is the entry point of the Logging system.
It exposes three macro definitions to ease its usage: ``logInfo``, ``logWarning`` and ``logError``.
Moreover, it allows the definition of new categories, in addition to those already available
(:class:`INFO_MSG`, :class:`WARN_MSG` and :class:`ERROR_MSG`).
It provides filtering by category using regular expressions, as well as control of the verbosity of the Logging system.
Details of the possible Logging system configurations can be found in the :ref:`dds_layer_core_logging` section.


XML profiles configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS offers the possibility to make changes in its default settings by using XML profile configuration files.
Thus, the behavior of the DDS Entities can be modified without the need for the user to implement any program source
code or re-build an existing application.

The user has XML tags for each of the API functionalities.
Therefore, it is possible to build and configure DomainParticipant profiles through the ``<participant>`` tag, or
the DataWriter and DataReader profiles with the ``<data_writer>`` and ``<data_reader>`` tags respectively.

For a better understanding of how to write and use these XML profiles configuration files you can continue reading
the :ref:`xml_profiles` section.

Environment variables
^^^^^^^^^^^^^^^^^^^^^

Environment variables are those variables that are defined outside the scope of the program, through operating system
functionalities.
Fast DDS relies on two environment variables so that the user can easily customize the default settings of DDS
applications.
These two environment variables are as follows:

* ``FASTRTPS_DEFAULT_PROFILES_FILE``.
  Defines the location of the profile configuration XML files.

* ``ROS_DISCOVERY_SERVER``.
  Sets as the default discovery protocol the Server-Client Discovery.
  Through this environment variable, the user can provide a list of locators (IP-port pairs) that specify the addresses
  of the servers.
  In this case, the server can be a user configured DomainParticipant, or one created by a binary provided by Fast DDS.
  In this sense, the variable configures the DomainParticipants as clients to the servers in the list.

More information about Fast DDS environment variables can be found in the :ref:`env_vars` section.
