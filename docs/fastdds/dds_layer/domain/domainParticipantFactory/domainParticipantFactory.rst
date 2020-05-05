.. _dds_layer_domainParticiantFactory:

DomainParticipantFactory
========================

The sole purpose of this class is to allow the creation and destruction of DomainParticipant objects.
DomainParticipantFactory itself has no factory, it is a pre-existing singleton object that can be accessed
through the :func:`get_instance()` static method on the DomainParticipantFactory class.

The behavior of the DomainParticipantFactory can be modified with the QoS values
specified on :ref:`dds_layer_domainParticiantFactoryQos`.
Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the :func:`set_qos()` method.

DomainParticipantFactory does not accept any Listener, since it is not an Entity.

.. _dds_layer_domainParticiantFactoryQos:

DomainParticipantFactoryQos
---------------------------

DomainParticipantFactoryQos controls the behavior of the :ref:`dds_layer_domainParticiantFactory`.
Internally it contains the following QosPolicy objects:

+-------------------------------+------------------------+----------+
| Name                          | QosPolicy class        | Mutable  |
+===============================+========================+==========+
| ``entity_factory_``           | EntityFactoryQosPolicy | yes      |
+-------------------------------+------------------------+----------+

Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the
:func:`set_qos()` method.


.. _dds_layer_domainParticiantFactory_profiles:

Loading profiles from an XML file
---------------------------------

To create Entities based on an XML profile, the file containing such profile must be loaded first with the
:func:`load_XML_profiles_file()` method.
Once loaded, the name of the profile can be used to create Entities that will have QoS settings according to
the profile specifications.

