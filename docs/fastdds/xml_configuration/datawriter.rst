.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _publisherprofiles:

DataWriter profiles
-------------------

The DataWriter profiles allow for configuring |DataWriters| from an XML file.
These profiles are defined within the ``<data_writer>`` or ``<publisher>`` XML tags.
Thus, the following XML code snippets are equivalent.

+----------------------------------------------------------+-----------------------------------------------------------+
| **DataWriter profile** - Definition method 1             | **DataWriter profile** - Definition method 2              |
+----------------------------------------------------------+-----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               | .. literalinclude:: /../code/XMLTester.xml                |
|   :language: xml                                         |   :language: xml                                          |
|   :start-after: <!-->XML-DATAWRITER-COMPARISON<-->       |   :start-after: <!-->XML-PUBLISHER-COMPARISON<-->         |
|   :end-before: <!--><-->                                 |   :end-before: <!--><-->                                  |
+----------------------------------------------------------+-----------------------------------------------------------+

.. important::

    The ``<data_writer>`` and ``<publisher>`` XML tags are equivalent.
    Therefore, XML profiles in which the DataWriters are defined with the ``<publisher>``
    tag are fully compatible with *Fast DDS*.

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
     - Sets the name under which the ``<data_writer>`` profile is registered in the DDS Domain, |br|
       so that it can be loaded later by the |DomainParticipant|, as shown in |br|
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<data_writer>`` profile as the default profile. Thus, if a default profile |br|
       exists, it will be used when no other DataWriter profile is specified at the |br|
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
     - It configures some time related parameters |br| of the DataWriter.
     - :ref:`Times <pubtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. |br|
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. |br|
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<external_unicast_locators>``
     - List of :ref:`external_locators` |br|
       to announce for the communication |br|
       with this DataWriter.
     - :ref:`externalLocatorListType`
     -
   * - ``<ignore_non_matching_locators>``
     - Whether to ignore locators received on |br|
       announcements from other entities when |br|
       they don't match with any of the locators |br|
       announced by this DataWriter.
     - ``bool``
     - false
   * - ``<throughputController>``
     - Limits the output bandwidth of the |br|
       DataWriter.
     - :ref:`Throughput`
     -
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for DataWriter's |br|
       history. See :ref:`historyqospolicykind`.
     - :ref:`HistoryMemoryPolicy <memorymanagementpolicy>`
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
     - Sets the |RTPSEndpointQos::entity_id-api| of the |RTPSEndpointQos| |br| class.
     - ``int16_t``
     - -1
   * - ``<matchedSubscribersAllocation>``
     - Sets the limits of the collection of matched |br|
       DataReaders. See |br|
       :ref:`participantresourcelimitsqos`.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DATAWRITER<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-71, 73-74

.. note::

    - :class:`LOCATOR_LIST` means a :ref:`LocatorListType` is expected.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _pubtimes:

Times
""""""

+------------------------------+-------------------------------------------------------+---------------------+---------+
| Name                         | Description                                           | Values              | Default |
+==============================+=======================================================+=====================+=========+
| ``<initialHeartbeatDelay>``  | Initial heartbeat delay.                              | :ref:`DurationType` | 12 ms   |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<heartbeatPeriod>``        | Periodic heartbeat period.                            | :ref:`DurationType` | 3 s     |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<nackResponseDelay>``      | Delay to apply to the response of an ACKNACK message. | :ref:`DurationType` | 5 ms    |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<nackSupressionDuration>`` | This time allows the DataWriter to ignore NACK |br|   | :ref:`DurationType` | 0 ms    |
|                              | messages for a given period of time right after |br|  |                     |         |
|                              | the data has been sent.                               |                     |         |
+------------------------------+-------------------------------------------------------+---------------------+---------+

