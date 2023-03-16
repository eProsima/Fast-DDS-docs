.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _Topic:

Topic profiles
--------------

The topic profiles allow for configuring |Topic| from an XML file.
These profiles are defined within the ``<topic>`` XML tags.

Topic XML attributes
^^^^^^^^^^^^^^^^^^^^

The ``<topic>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``profile_name``
     - Sets the name under which the ``<topic>`` profile is registered in the DDS Domain, |br|
       so that it can be loaded later by the |DataWriter| or the |DataReader|
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<topic>`` profile as the default profile. Thus, if a default profile |br|
       exists, it will be used when no other Topic profile is specified at the |br|
       Topic's creation.
     - Optional

Topic configuration
^^^^^^^^^^^^^^^^^^^

This XML element allows the configuration of the :ref:`historyqospolicy` and :ref:`resourcelimitsqospolicy` as
the default QoS for any entity that contains those policies.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
  * - ``<historyQos>``
    - It controls the behavior of *Fast DDS* |br|
      when the value of an instance changes  |br|
      before it is finally communicated to |br|
      some of its existing DataReaders. |br|
    - :ref:`hQos`
  * - ``<resourceLimitsQos>``
    - It controls the resources that *Fast DDS* |br|
      can use in order to meet the |br|
      requirements imposed by the application |br|
      and other QoS settings.
    - :ref:`rLsQos`

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC-PROFILE<-->
    :end-before: <!--><-->
