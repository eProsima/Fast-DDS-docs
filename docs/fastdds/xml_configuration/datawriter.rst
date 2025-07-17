.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _publisherprofiles:

DataWriter profiles
-------------------

The DataWriter profiles allow for configuring |DataWriters| from an XML file.
These profiles are defined within the ``<data_writer>`` XML tags.

DataWriter XML attributes
^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<data_writer>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``profile_name``
     - Sets the name under which the ``<data_writer>`` profile is registered in the DDS Domain,
       so that it can be loaded later by the |DomainParticipant|, as shown in
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<data_writer>`` profile as the default profile. Thus, if a default profile
       exists, it will be used when no other DataWriter profile is specified at the
       DataWriter's creation.
     - Optional

DataWriter configuration
^^^^^^^^^^^^^^^^^^^^^^^^

The DataWriter configuration is performed through the XML elements listed in the following table.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<topic>``
     - :ref:`TopicType` configuration of the DataWriter.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - DataWriter :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It configures some time related parameters of the DataWriter.
     - :ref:`WriterTimes <pubtimes>`
     -
   * - ``<transport_priority>``
     - It configures the :ref:`transport priority <transportpriorityqospolicy>` of the DataWriter.
     - ``int32_t``
     - 0
   * - ``<unicastLocatorList>``
     - List of input unicast locators.
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators.
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<external_unicast_locators>``
     - List of :ref:`external_locators`
       to announce for the communication
       with this DataWriter.
     - :ref:`externalLocatorListType`
     -
   * - ``<ignore_non_matching_locators>``
     - Whether to ignore locators received on
       announcements from other entities when
       they don't match with any of the locators
       announced by this DataWriter.
     - ``bool``
     - false
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for DataWriter's
       history. See :ref:`historyqospolicykind`.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for |EDPStatic|.
     - ``int16_t``
     - -1
   * - ``<entityID>``
     - Sets the |RTPSEndpointQos::entity_id-api| of the |RTPSEndpointQos|  class.
     - ``int16_t``
     - -1
   * - ``<matchedSubscribersAllocation>``
     - Sets the limits of the collection of matched
       DataReaders. See
       :ref:`participantresourcelimitsqos`.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DATAWRITER<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-86, 88-89

.. note::

    - :class:`LOCATOR_LIST` means a :ref:`LocatorListType` is expected.

    - :class:`EXTERNAL_LOCATOR_LIST` means a :ref:`externalLocatorListType` is expected.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _pubtimes:

WriterTimes
"""""""""""

These parameters are included within :ref:`rtpsreliablewriterqos` in the :ref:`writertimes` structure.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<initial_heartbeat_delay>``
     - Initial heartbeat delay.
     - :ref:`DurationType`
     - 12 ms
   * - ``<heartbeat_period>``
     - Periodic heartbeat period.
     - :ref:`DurationType`
     - 3 s
   * - ``<nack_response_delay>``
     - Delay to apply to the response of an ACKNACK message.
     - :ref:`DurationType`
     - 5 ms
   * - ``<nack_supression_duration>``
     - This time allows the DataWriter to ignore NACK
       messages for a given period of time right after
       the data has been sent.
     - :ref:`DurationType`
     - 0 ms
