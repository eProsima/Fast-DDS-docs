.. _dds_layer_domainParticipantFactory:

DomainParticipantFactory
========================

The sole purpose of this class is to allow the creation and destruction of :ref:`dds_layer_domainParticipant` objects.
DomainParticipantFactory itself has no factory, it is a singleton object that can be accessed
through the :func:`get_instance()` static method on the DomainParticipantFactory class.

The behavior of the DomainParticipantFactory can be modified with the QoS values
specified on :ref:`dds_layer_domainParticipantFactoryQos`.
Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the :func:`set_qos()` method.

DomainParticipantFactory does not accept any Listener, since it is not an Entity.


.. _dds_layer_domainParticipantFactoryQos:

DomainParticipantFactoryQos
---------------------------

DomainParticipantFactoryQos controls the behavior of the :ref:`dds_layer_domainParticipantFactory`.
Internally it contains the following QosPolicy objects:

+-------------------------------+------------------------+----------+
| Name                          | QosPolicy class        | Mutable  |
+===============================+========================+==========+
| ``entity_factory_``           | EntityFactoryQosPolicy | yes      |
+-------------------------------+------------------------+----------+

Since the :ref:`dds_layer_domainParticipantFactory` is a singleton, its QoS can only be modified with the
:func:`set_qos()` method.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTFACTORYQOS
   :end-before: //!

.. _dds_layer_domainParticipantFactory_profiles:


.. _dds_layer_domainParticipantFactory_load_profiles:

Loading profiles from an XML file
---------------------------------

To create Entities based on XML profiles, the file containing such profiles must be loaded first.

If the profile is described in one of the default loaded files, it will be automatically available on initialization.
Otherwise, :func:`load_XML_profiles_file()` method can be used to load the profiles in the XML.
See section :ref:`xml_profiles` for more information regarding XML profile format and automatic loading.

Once loaded, the name of the profiles can be used to create Entities that will have QoS settings according to
the profile specifications.



