.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _persistence_service:

Persistence Service
===================

Using default QoS, the |DataWriter-api| history is only available for |DataReaders-api| throughout the |DataWriter-api|
life.
This means that the history does not persist between |DataWriter-api| initializations and therefore it is on a empty
state on |DataWriter-api| creation.
Similarly, |DataReader-api| history does not persist the |DataReader-api| life, thus also being empty on
|DataReader-api| creation.
However, *eProsima Fast DDS* offers the possibility to configure the |DataWriter-api| history to be stored in a
persistent database, so that the |DataWriter-api| can load it on creation.
Furthermore, |DataReader-api| can be configured to stored the last notified change in the database, so that they can
recover their state on creation.

This mechanism allows to recover a previous state on starting the Data Distribution Service, thus adding robustness to
applications in the case of, for example, unexpected shutdowns.
Configuring the persistence service, |DataWriters-api| and |DataReaders-api| can resume their operation from the state
in which they where when the shutdown occurred.

.. note::
    Mind that |DataReaders-api| do not stored their history into the database, but rather the last notified change from
    the |DataWriter-api|.
    This means that they will resume operation where they left, but they will not have the previous information, since
    that was already notified to the application.


.. _persistence_service_conf:

Configuration
-------------

The configuration of the persistence service is accomplished through the setting of the appropriate |DataWriter-api|
and |DataReader-api| |DurabilityQosPolicy-api|, and by specifying the appropriate properties in the entities'
(|DomainParticipant-api|, |DataWriter-api|, or |DataReader-api|) |PropertyPolicyQos-api|.

* For the :ref:`persistence_service` to have any effect, the |DurabilityQosPolicyKind-api| needs to be set to
  |TRANSIENT_DURABILITY_QOS-api|.

* A persistence identifier (|Guid_t-api|) must be set for the entity using the property ``dds.persistence.guid``.
  This identifier is used to load the appropriate data from the database, and also to synchronize |DataWriter-api| and
  |DataReader-api| between restarts.
  The |Guid_t-api| consists on 16 octets separated in two groups:

    * The first 12 octets correspond to the |GuidPrefix_t-api|.
    * The last 4 octets correspond to the |EntityId_t-api|.

  The persistence identifier is specified using a string of 12 dot-separated octets, expressed in hexadecimal base,
  followed by a ``|`` separator and another 4 dot-separated octets, also expressed in hexadecimal base (see
  :ref:`persistence_example`).
  For selecting an appropriate |Guid_t-api| for the |DataReader-api| and |DataWriter-api|, please refer to
  `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (section *9.3.1 The Globally Unique Identifier (GUID)*).

* A persistence plugin must be configured for managing the database using property ``dds.persistence.plugin`` (see
  :ref:`persistence_builtin_plugins`):


.. _persistence_builtin_plugins:

Built-in plugins
----------------

*eProsima Fast DDS* provides one builtin plugin for the persistence service: :ref:`persistence-sqlite3`.


.. _persistence-sqlite3:

PERSISTENCE:SQLITE3
^^^^^^^^^^^^^^^^^^^

This plugin provides persistence through a local database file using *SQLite3* API.

To activate the plugin, ``dds.persistence.plugin`` property must be added to the |PropertyPolicyQos-api| of the
|DomainParticipant-api|, |DataWriter-api|, or |DataReader-api| with value ``builtin.SQLITE3``.
Furthermore, ``dds.persistence.sqlite3.filename`` property must be added to the entities |PropertyPolicyQos-api|,
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

.. important::
    The plugin set in the |PropertyPolicyQos-api| of |DomainParticipant-api| only applies if that of the
    |DataWriter-api|/|DataReader-api| does no exist or is invalid.


.. _persistence_example:

Example
-------

This example shows how to configure the persistence service using :ref:`persistence-sqlite3` plugin.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE
    :end-before: //!--
    :dedent: 4
