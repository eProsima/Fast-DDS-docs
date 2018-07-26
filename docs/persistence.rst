.. |br| raw:: html

   <br />

.. _persistence:

Persistence
===========

By default, writer's history is available for remote readers throughout writer's life. 
You can configure Fast RTPS to provide persistence between application executions. 
When a writer is created again, it will maintain the previous history and a new remote reader will receive all 
samples sent by the writer throughout its life.

A reader keeps information on the latest change notified to the user for each matching writer.
Persisting this information, you could save bandwith, as the reader will not ask the writers for changes already notified.

In summary, enabling this feature you will protect the state of endpoints against unexpected failures, 
as they will continue communicating after being restarted as if they were just disconnected from the network.

Imagine, for instance, that a writer with a policy to keep its last 100 samples has its history full of changes and 
the machine where it runs has a power failure. 
When the writer is started again, if a new reader is created, it will not receive the 100 samples that were on the history of the writer. 
With persistence enabled, changes in the history of the writer will be written to disk, and read again when the writer is restarted.

With readers, the information written to disk is different. 
Only information about the last change notified to the user is stored on disk.
When a persistent reader is restarted, it will load this information, and will only ask the matching writers to 
resend those changes that were not notified to the upper layers.

**persistence_guid**

Whenever an endpoint (reader or writer) is created, a unique identifier (GUID) is generated. 
If the endpoint is restarted, a new GUID will be generated, and other endpoints won't be able to know it was the same one.
For this reason, a specific parameter persistence_guid should be configured on :class:`eprosima::fastrtps::rtps::EndpointAttributes`.
This parameter will be used as the primary key of the data saved on disk, and will also be used to identify the endpoint on the DDS domain.

Configuration
-------------

We recommend you to look at the example of how to use this feature the distribution comes with while reading
this section. It is located in `examples/RTPSTest_persistent`

In order for the persistence feature to work, some specific :class:`eprosima::fastrtps::rtps::Writer` or
:class:`eprosima::fastrtps::rtps::Reader` attributes should be set:

* ``durabilityKind`` should be set to ``TRANSIENT``
* ``persistence_guid`` should not be all zeros
* A persistence plugin should be configured either on the :class:`eprosima::fastrtps::rtps::Writer`, the :class:`eprosima::fastrtps::rtps::Reader` or the :class:`eprosima::fastrtps::rtps::RTPSParticipant`

You can select and configure the persistence plugin through :class:`eprosima::fastrtps::rtps::RTPSParticipant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Throughout this page there are tables showing you the properties used by each persistence plugin.

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

