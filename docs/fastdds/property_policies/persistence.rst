.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_persistence:

Persistence Service Settings
----------------------------

When using the :ref:`persistence_service`, the following properties must be set on the
|DomainParticipant-api|, |DataWriter-api|, or |DataReader-api| via their |PropertyPolicyQos|.

.. important::

   It is strongly recommended to define all three properties at the endpoint
   (|DataWriter-api| or |DataReader-api|) level for predictable behaviour.
   ``dds.persistence.plugin`` and ``dds.persistence.sqlite3.filename`` are resolved
   as a pair from the first level where ``dds.persistence.plugin`` is present
   (endpoint first, then |DomainParticipant-api|); a ``dds.persistence.sqlite3.filename``
   defined at the endpoint level without a ``dds.persistence.plugin`` at the same level
   is ignored.
   ``dds.persistence.guid`` must be defined at the endpoint level; if absent, the behaviour
   falls back to |TRANSIENT_LOCAL_DURABILITY_QOS-api|, regardless of any plugin configuration.

* Property ``dds.persistence.plugin`` selects the persistence plugin to use.
  It can be set on the |DomainParticipant-api|, a |DataWriter-api|, or a |DataReader-api|.
  If this property is absent at the endpoint level, both ``dds.persistence.plugin`` and
  ``dds.persistence.sqlite3.filename`` are taken from the |DomainParticipant-api| instead.
  Setting it at the endpoint level is strongly recommended.
  At the moment, the only valid value is ``builtin.SQLITE3``
  (see :ref:`persistence_sqlite3_builtin_plugin`).

* Property ``dds.persistence.sqlite3.filename`` specifies the path to the SQLite3 database
  file used for persistent storage when the SQLite3 plugin is used.
  This property is optional; the default value is ``persistence.db``.
  To avoid undesired delays caused by concurrent access, it is advisable to use a different
  database file for each DataWriter and DataReader.

* Property ``dds.persistence.guid`` sets the persistence identifier (|Guid_t-api|) for a
  DataWriter or DataReader.
  This identifier is used to load the appropriate data from the database and to synchronize
  the entity between restarts.
  This property must be defined at the endpoint level.
  If absent, the durability behaviour falls back to |TRANSIENT_LOCAL_DURABILITY_QOS-api|,
  regardless of any plugin configuration.

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
       :lines: 2-4, 6-96, 98-99
