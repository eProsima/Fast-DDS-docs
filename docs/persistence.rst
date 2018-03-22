.. |br| raw:: html

   <br />

.. _persistence:

Persistence
===========

Fast RTPS can be configured to provide persistence to the history of a writer 
and the highest sequence number notified by a reader.

We recommend you to look at the example of how to use this feature the distribution comes with while reading
this section. It is located in `examples/RTPSTest_persistent`

You can select and configure the persistence plugin through :class:`eprosima::fastrtps::rtps::RTPSParticipant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Throughout this page there are tables showing you the properties used by each persistence plugin.

Configuration
-------------

In order for the persistence service to work, some specific :class:`eprosima::fastrtps::rtps::Writer` or
:class:`eprosima::fastrtps::rtps::Reader` attributes should be set:

* ``durabilityKind`` should be set to ``TRANSIENT``
* ``persistence_guid`` should not be all zeros
* A persistence plugin should be configured either on the :class:`eprosima::fastrtps::rtps::Writer`, the :class:`eprosima::fastrtps::rtps::Reader` or the :class:`eprosima::fastrtps::rtps::RTPSParticipant`

Built-in plugins
----------------

Current version comes out with one persistence built-in plugin:

* **SQLITE3**: this plugin provides persistence on a local file using SQLite3 API.

.. _persistence-sqlite3:

PERSISTENCE:SQLITE3
^^^^^^^^^^^^^^^^^^^

This built-in plugin provides persistence on a local file using SQLite3 API.

You can activate this plugin using RTPSParticipant, Reader or Writer property ``dds.persistence.plugin`` with the value ``builtin.SQLITE3``.
Next table shows you the properties used by this persistence plugin.

.. list-table:: **Properties to configure Persistence::SQLITE3**
   :header-rows: 1
   :align: left

   * - Property name |br|
       (all properties have "dds.persistence.sqlite3." prefix)
     - Property value
   * - filename
     - Name of the file used for persistent storage. |br|
       Default value: *persistence.db*

Example
^^^^^^^

This example shows you how to configure a RTPSParticipant to activate and configure :ref:`persistence-sqlite3` plugin.
It also configures a Writer to persist its history on local storage, and a Reader to persist the highest notified
sequence number on local storage.

**RTPSParticipant attributes**

.. code-block:: c++

   eprosima::fastrtps::rtps::RTPSParticipantAttributes part_attr;

   // Activate Persistence:SQLITE3 plugin
   part_attr.properties.properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");

   // Configure Persistence:SQLITE3 plugin
   part_attr.properties.properties().emplace_back("dds.persistence.sqlite3.filename", "example.db");

**Writer attributes**

.. code-block:: c++

   eprosima::fastrtps::rtps::WriterAttributes writer_attr;

   // Set durability to TRANSIENT
   writer_attr.endpoint.durabilityKind = TRANSIENT;

   // Set persistence_guid
   writer_attr.endpoint.persistence_guid.guidPrefix.value[11] = 1;
   writer_attr.endpoint.persistence_guid.entityId = 0x12345678;

**Reader attributes**

.. code-block:: c++

   eprosima::fastrtps::rtps::ReaderAttributes reader_attr;

   // Set durability to TRANSIENT
   reader_attr.endpoint.durabilityKind = TRANSIENT;

   // Set persistence_guid
   reader_attr.endpoint.persistence_guid.guidPrefix.value[11] = 1;
   reader_attr.endpoint.persistence_guid.entityId = 0x3456789A;

