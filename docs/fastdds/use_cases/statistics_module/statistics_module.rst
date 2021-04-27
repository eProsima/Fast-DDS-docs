.. include:: ../../../03-exports/aliases-api.include

.. _statistics_module:

Statistics module
=================

*eProsima Fast DDS* :ref:`statistics` allows the user to monitor the data being exchanged by its application.
In order to use this module, the user must enable it in the monitored application, and create another application that
receives the data being published by the statistics DataWriters.
The user can also use for the latter the
`*eProsima Fast DDS Statistics Backend* <https://fast-dds-statistics-backend.readthedocs.io/en/latest/>`_ which already
implements the collection and aggregation of the data coming from the statistics topics.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Enable Statistics module
------------------------

The Statistics module has to be enabled both at build and runtime.
On the one hand, :ref:`CMake option <cmake_options>` ``FASTDDS_STATISTICS`` must be enabled when building the library.
On the other hand, the desired statistics DataWriters should be enabled using the :ref:`statistics_dds_layer`.

The statistics DataWriters can be enabled automatically using the :ref:`propertypolicyqos` ``fastdds.statistics`` and
the :ref:`env_vars_fastdds_statistics` environment variable.
They can also be enabled manually following the next example:

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: // ENABLE_DISABLE_STATISTICS_DATAWRITER
   :end-before: //!--
   :dedent: 8

Create monitoring application
-----------------------------

Once the monitored application is publishing the collected data within the statistics topics enabled by the user,
another application should be configured to subscribe to those topics.
This application is a DDS standard application where the statistics DataReaders should be created.
In order to create these statistics DataReaders, the user should follow the next steps:

* Using the `statistics IDL <https://github.com/eProsima/Fast-DDS/blob/master/include/fastdds/statistics/types.idl>`_
  provided in the public API, generate the |TopicDataTypes-api| with :ref:`Fast DDS-Gen <fastddsgen_usage>`.

* Create the |DomainParticipant-api| and register the |TopicDataTypes-api| and the corresponding statistics
  |Topics-api|.

* Create the statistics DataReaders using the corresponding statistics topic.
