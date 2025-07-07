.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _participantfactoryprofiles:

DomainParticipantFactory profiles
---------------------------------

The |DomainParticipantFactory| profiles allow the definition of the configuration of |DomainParticipantFactory-api|
through XML files.
These profiles are defined within the ``<domainparticipant_factory>`` XML tags.

.. _domainparticipantfactoryattributes:

DomainParticipantFactory XML attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<domainparticipant_factory>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``profile_name``
     - Sets the name under which the ``<domainparticipant_factory>`` profile is registered in the 
       DDS Domain, so that it can be loaded later by the |DomainParticipantFactory-api|, as shown in 
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<domainparticipant_factory>`` profile as the default profile. Thus, if a default 
       profile exists, it will be used when creating the DomainParticipantFactory
     - Optional

.. _domainparticipantfactoryconfig:

DomainParticipantFactory configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<domainparticipant_factory>`` element has the following children elements:

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
   * - ``<qos>``
     - DomainParticipantFactory QoS.
     - :ref:`domainparticipantfactoryqosconfig`

.. _domainparticipantfactoryqosconfig:

QoS element type
""""""""""""""""

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
   * - ``<entity_factory>``
     - Entity factory QoS Policy.
     - :ref:`xml_entity_factory`
   * - ``<shm_watchdog_thread>``
     - |ThreadSettings| for the SHM watchdog thread. 
       See :ref:`concurrency_multithreading`.
     - :ref:`ThreadSettingsType`
   * - ``<file_watch_threads>``
     - |ThreadSettings| for the File watch threads.
       See :ref:`concurrency_multithreading`.
     - :ref:`ThreadSettingsType`

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT-FACTORY<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-24, 26-27
