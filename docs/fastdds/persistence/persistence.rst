.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _persistence_service:

Persistence Service
===================

Using default QoS, the :ref:`dds_layer_publisher_dataWriter` history is only available for
:ref:`dds_layer_subscriber_dataReader` throughout the DataWriter's life.
This means that the history does not persist between DataWriter initializations and therefore it is on an empty
state on DataWriter creation.
Similarly, the DataReader history does not persist the DataReader's life, thus also being empty on
DataReader creation.
However, *eProsima Fast DDS* offers the possibility to configure the DataWriter's history to be stored in a
persistent database, so that the DataWriter can load its history from it on creation.
Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can
recover their state on creation.

This mechanism allows recovering a previous state on starting the Data Distribution Service, thus adding robustness to
applications in the case of, for example, unexpected shutdowns.
Configuring the persistence service, DataWriters and DataReaders can resume their operation from the state
in which they were when the shutdown occurred.

.. note::
    Mind that DataReaders do not store their history into the database, but rather the last notified change from
    the DataWriter.
    This means that they will resume operation where they left, but they will not have the previous information, since
    that was already notified to the application.


.. _persistence_service_conf:

Configuration
-------------

The configuration of the persistence service is accomplished by setting of the appropriate DataWriter and DataReader
|DurabilityQosPolicy|, and by specifying the suitable properties for each entity's (|DomainParticipant-api|, DataWriter,
or DataReader) |PropertyPolicyQos|.

* For the :ref:`persistence_service` to have any effect, the |DurabilityQosPolicyKind-api| needs to be set to
  |TRANSIENT_DURABILITY_QOS-api| or |PERSISTENT_DURABILITY_QOS-api|.

* A persistence identifier (|Guid_t-api|) must be set for the entity using the property ``dds.persistence.guid``.
  This identifier is used to load the appropriate data from the database, and also to synchronize DataWriter and
  DataReader between restarts.
  The GUID consists of 16 bytes separated into two groups:

    * The first 12 bytes correspond to the |GuidPrefix_t-api|.
    * The last 4 bytes correspond to the |EntityId_t-api|.

  The persistence identifier is specified using a string of 12 dot-separated bytes, expressed in hexadecimal base,
  followed by a vertical bar separator (``|``) and another 4 dot-separated bytes, also expressed in hexadecimal base
  (see :ref:`persistence_example`).
  For selecting an appropriate GUID for the DataReader and DataWriter, please refer to
  `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (section *9.3.1 The Globally Unique Identifier (GUID)*).

  If no ``dds.persistence.guid`` is specified,
  the durability behavior will fallback to |TRANSIENT_LOCAL_DURABILITY_QOS-api|.

* A persistence plugin must be configured for managing the database using property ``dds.persistence.plugin`` (see
  :ref:`persistence_sqlite3_builtin_plugin`):

.. _persistence_sqlite3_builtin_plugin:

PERSISTENCE:SQLITE3 built-in plugin
-----------------------------------

This plugin provides persistence through a local database file using *SQLite3* API.
To activate the plugin, ``dds.persistence.plugin`` property must be added to the PropertyPolicyQos of the
DomainParticipant, DataWriter, or DataReader with value ``builtin.SQLITE3``.
Furthermore, ``dds.persistence.sqlite3.filename`` property must be added to the entities PropertyPolicyQos,
specifying the database file name.
These properties are summarized in the following table:

.. list-table:: **Persistence::SQLITE3 configuration properties**
   :header-rows: 1
   :align: left

   * - Property name
     - Property value
   * - ``dds.persistence.plugin``
     - ``builtin.SQLITE3``
   * - ``dds.persistence.sqlite3.filename``
     - Name of the file used for persistent storage. |br|
       Default value: ``persistence.db``

.. note::
    To avoid undesired delays caused by concurrent access to the SQLite3 database, it is advisable to specify a
    different database file for each DataWriter and DataReader.

.. important::
    The plugin set in the PropertyPolicyQos of DomainParticipant only applies if that of the
    DataWriter/DataReader does no exist or is invalid.


.. _persistence_example:

Example
-------

This example shows how to configure the persistence service using :ref:`persistence_sqlite3_builtin_plugin` plugin both
from C++ and using *eProsima Fast DDS* XML profile files (see :ref:`xml_profiles`).

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE
       :end-before: //!--
       :dedent: 4

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE<-->
       :end-before: <!--><-->
       :lines: 2-4, 6-61, 63-64

.. note::
    For instructions on how to create DomainParticipants, DataReaders, and DataWriters, please refer to
    :ref:`dds_layer_domainParticipant_creation_profile`, :ref:`dds_layer_publisher_datawriter_creation_profile`, and
    :ref:`dds_layer_subscriber_datareader_creation_profile` respectively.
