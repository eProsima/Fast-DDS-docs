.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_persistence:

Persistence Service Settings
----------------------------

When using the :ref:`persistence_service`, specific properties must be set on the
|DomainParticipant-api|, |DataWriter-api|, or |DataReader-api| via their |PropertyPolicyQos|.

* Property ``dds.persistence.plugin`` selects the persistence plugin to use.
  It can be set on the |DomainParticipant-api|, a |DataWriter-api|, or a |DataReader-api|.
  When set on the |DomainParticipant-api|, it acts as the fallback for any DataWriter or
  DataReader that does not specify its own plugin.
  At the moment, the only valid value is ``builtin.SQLITE3``
  (see :ref:`persistence_sqlite3_builtin_plugin`).

* Property ``dds.persistence.sqlite3.filename`` specifies the path to the SQLite3 database
  file used for persistent storage when the SQLite3 plugin is used.
  The default value is ``persistence.db``.
  To avoid undesired delays caused by concurrent access, it is advisable to use a different
  database file for each DataWriter and DataReader.

* Property ``dds.persistence.guid`` sets the persistence identifier (|Guid_t-api|) for a
  DataWriter or DataReader.
  This identifier is used to load the appropriate data from the database and to synchronize
  the entity between restarts.
  If this property is not set, the durability behaviour falls back to
  |TRANSIENT_LOCAL_DURABILITY_QOS-api|.

The following example shows how to configure the persistence service properties along with the
required QoS settings for a correct operation of the service (see :ref:`persistence_example`).

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE
       :end-before: //!--
       :dedent: 4

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE<-->
       :end-before: <!--><-->
       :lines: 2-4, 6-61, 63-64
