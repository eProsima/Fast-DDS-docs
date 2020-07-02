.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipantFactory:

DomainParticipantFactory
========================

The sole purpose of this class is to allow the creation and destruction of :ref:`dds_layer_domainParticipant` objects.
|DomainParticipantFactory-api| itself has no factory, it is a singleton object that can be accessed
through the |DomainParticipantFactory::get_instance-api| static member function on the |DomainParticipantFactory-api|
class.

The behavior of the DomainParticipantFactory can be modified with the QoS values
specified on :ref:`dds_layer_domainParticipantFactoryQos`.
Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the
|DomainParticipantFactory::set_qos-api| member function.

DomainParticipantFactory does not accept any Listener, since it is not an Entity.


.. _dds_layer_domainParticipantFactoryQos:

DomainParticipantFactoryQos
---------------------------

DomainParticipantFactoryQos controls the behavior of the :ref:`dds_layer_domainParticipantFactory`.
Internally it contains the following |QosPolicy-api| objects:

+------------------------------+----------------------------------------------------------------------------+----------+
| QosPolicy class              | Accessor/Mutator                                                           | Mutable  |
+==============================+============================================================================+==========+
| |EntityFactoryQosPolicy|     | |DomainParticipantFactoryQos::entity_factory-api|                          | Yes      |
+------------------------------+----------------------------------------------------------------------------+----------+

Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the
|DomainParticipantFactory::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTFACTORYQOS
   :end-before: //!
   :dedent: 8

.. _dds_layer_domainParticipantFactory_profiles:


.. _dds_layer_domainParticipantFactory_load_profiles:

Loading profiles from an XML file
---------------------------------

To create Entities based on XML profiles, the file containing such profiles must be loaded first.

If the profile is described in one of the default loaded files, it will be automatically available on initialization.
Otherwise, |DomainParticipantFactory::load_XML_profiles_file-api| member function can be used to load the profiles in
the XML.
See section :ref:`xml_profiles` for more information regarding XML profile format and automatic loading.

Once loaded, the name of the profiles can be used to create Entities that will have QoS settings according to
the profile specifications.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_LOAD_XML_PROFILE
   :end-before: //!
   :dedent: 8




