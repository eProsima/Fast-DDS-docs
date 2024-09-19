.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_statistics_module_questions:

Statistics Module Frequently Asked Questions
============================================

.. collapse::  What is the purpose of the statistics module?

    |br|

    The Fast DDS Statistics module is an extension of Fast DDS that enables the recollection of data concerning the DDS communication. For further information, see :ref:`statistics`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the statistics module work?

    |br|

    The collected data is published using DDS over dedicated topics using builtin DataWriters within the Statistics module. For further information, see :ref:`statistics_domainparticipant`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the consequence of compiling the statistics module?

    |br|

    It may entail affecting the application's performance. For further information, see :ref:`statistics`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How can we activate the statistics module?

    |br|

    It can be activated using the ``-DFASTDDS_STATISTICS=ON`` at CMake configuration step. For further information, see :ref:`statistics`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How can we start collecting data?

    |br|

    In order to start collecting data in one of the statistics topics, the corresponding statistics DataWriter should be enabled. For further information, see :ref:`statistics_domainparticipant`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How can we enable the statistics DataWriter?

    |br|

    It can either be done automatically or be enabled at run time using one of two methods: ``enable_statistics_datawriter()`` or ``enable_statistics_datawriter_with_profile()``. For further information, see :ref:`statistics_enable_datawriters`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is the environment variable ``FASTDDS_STATISTICS`` used?

    |br|

    The environment variable is only used in the case where the CMake option ``FASTDDS_STATISTICS`` has been enabled. In any other case, the environment variable has no effect. The statistics DataWriters that will be enabled when the |DomainParticipant-api| is enabled would be the union between those specified in the ``properties()`` ``fastdds.statistics`` and those included with the environment variable. For further information, see :ref:`env_vars_fastdds_statistics`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the monitor service?

    |br|

    The Monitor Service targets any application implementing the subscription side of the Monitor Service Status Topic, giving the possibility of retrieving the Monitoring Information of the local entities. For further information, see :ref:`monitor_intro`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What Information can the monitor service carry?

    |br|

    It can carry information about the monitoring information of the local entities of a particular DomainParticipant. For further information, see :ref:`monitor_intro`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How can the monitor service be activated?

    |br|

    The Monitor Service can be activated using the ``-DFASTDDS_STATISTICS=ON`` at CMake configuration step. For further information, see :ref:`monitor_service_configuration`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  At which layers can the monitor service be enabled?

    |br|

    It can be programmatically enabled in both DDS Layer and RTPS Layer through the ``enable_monitor_service()`` and ``disable_monitor_service()`` calls. For further information, see :ref:`monitor_service_configuration`.

|
