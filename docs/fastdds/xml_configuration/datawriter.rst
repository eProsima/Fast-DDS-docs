.. include:: includes/aliases.rst

.. _publisherprofiles:

DataWriter profiles
-------------------

The DataWriter profiles allow declaring |DataWriters| from an XML file.
These profiles are defined within the ``<data_writer>`` or ``<publisher>`` XML tags.
Thus, the following XML codes are equivalent.

+----------------------------------------------------------+-----------------------------------------------------------+
| **DataWriter profile** - Definition method 1             | **DataWriter profile** - Definition method 2              |
+----------------------------------------------------------+-----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               | .. literalinclude:: /../code/XMLTester.xml                |
|   :language: xml                                         |   :language: xml                                          |
|   :start-after: <!-->XML-DATAWRITER-COMPARISON<-->       |   :start-after: <!-->XML-PUBLISHER-COMPARISON<-->         |
|   :end-before: <!--><-->                                 |   :end-before: <!--><-->                                  |
+----------------------------------------------------------+-----------------------------------------------------------+

.. Note::

    The ``<publisher>`` and ``<data_writer>`` XML tags are equivalent.
    **Therefore, XML profiles in which the |DataWriters| are defined with the ``<publisher>`` tag are fully compatible
    with Fast DDS**.

The ``<data_writer>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.
Attribute ``profile_name`` is a mandatory attribute and sets the name under which the ``<data_writer>`` profile is
registered in the DDS Domain, so that it can be loaded later by a |DomainParticipant|, as shown in
:ref:`loadingapplyingprofiles`.
The second attribute, ``is_default_profile``, sets the ``<data_writer>`` profile as the default profile. It is an
attribute.

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
     - List of input unicast locators. |br| It expects a :ref:`LocatorListType`.
     - ``Locator``
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. |br| It expects a :ref:`LocatorListType`.
     - ``Locator``
     -
   * - ``<throughputController>``
     - Limits the output bandwidth of the |br| DataWriter.
     - :ref:`Throughput`
     -
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for DataWriter's |br| history. See :ref:`historyqospolicykind`.
     - :ref:`HistoryMemoryPolicy <mempol>`
     - :class:`PREALLOCATED`
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for |EDPStatic|.
     - ``Int16``
     - -1
   * - ``<entityID>``
     - Set the |entity_id| of the |RTPSEndpointQos| |br| class.
     - ``Int16``
     - -1
   * - ``<matchedSubscribersAllocation>``
     - DataWriter :ref:`CommonAlloc` |br| related to the number of matched DataReaders.
     - :ref:`CommonAlloc`
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DATAWRITER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. _pubtimes:

Times
^^^^^

+------------------------------+-------------------------------------------------------+---------------------+---------+
| Name                         | Description                                           | Values              | Default |
+==============================+=======================================================+=====================+=========+
| ``<initialHeartbeatDelay>``  | Initial heartbeat delay.                              | :ref:`DurationType` | ~45 ms  |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<heartbeatPeriod>``        | Periodic heartbeat period.                            | :ref:`DurationType` | 3 s     |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<nackResponseDelay>``      | Delay to apply to the response of a ACKNACK message.  | :ref:`DurationType` | ~45 ms  |
+------------------------------+-------------------------------------------------------+---------------------+---------+
| ``<nackSupressionDuration>`` | This time allows the DataWriter to ignore NACK |br|   | :ref:`DurationType` | 0 ms    |
|                              | messages for a given period of time right after |br|  |                     |         |
|                              | the data has been sent.                               |                     |         |
+------------------------------+-------------------------------------------------------+---------------------+---------+

