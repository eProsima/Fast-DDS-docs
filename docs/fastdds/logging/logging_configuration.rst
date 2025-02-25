.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_config:

Module Configuration
--------------------

The logging module offers a variety of configuration options.
The different components of a log entry (see :ref:`dds_layer_log_logging_spec`) can be configured as explained in
:ref:`dds_layer_log_config_entry`.
Furthermore, the logging module allows for registering several log consumer, allowing applications to direct the
logging output to different destinations (see :ref:`dds_layer_log_register_consumers`).
In addition, some of the logging features can be configured using *eProsima Fast DDS* XML configuration files (see
:ref:`dds_layer_log_xml`).

.. _dds_layer_log_config_entry:

Log Entry
^^^^^^^^^

All the different components of a log entry are summarized in the following table (please refer to each component's
section for further explanation):

+--------------------------------------+----------+----------+
| Component                            | Optional | Default  |
+======================================+==========+==========+
| :ref:`dds_layer_log_timestamp`       | NO       | ENABLED  |
+--------------------------------------+----------+----------+
| :ref:`dds_layer_log_category`        | NO       | ENABLED  |
+--------------------------------------+----------+----------+
| :ref:`dds_layer_log_verbosity_level` | NO       | ENABLED  |
+--------------------------------------+----------+----------+
| :ref:`dds_layer_log_message`         | NO       | ENABLED  |
+--------------------------------------+----------+----------+
| :ref:`dds_layer_log_file_context`    | YES      | DISABLED |
+--------------------------------------+----------+----------+
| :ref:`dds_layer_log_function_name`   | YES      | ENABLED  |
+--------------------------------------+----------+----------+


.. _dds_layer_log_timestamp:

Timestamp
"""""""""

The log timestamp follows the `ISO 8601 standard <https://www.iso.org/iso-8601-date-and-time-format.html>`_ for local
timestamps, i.e. *YYYY-MM-DD hh:mm:ss.sss*.
This component cannot be further configured or disabled.


.. _dds_layer_log_category:

Category
""""""""

Log entries have a category assigned when producing the log via the macros presented in
:ref:`dds_layer_log_logging`.
The category component can be used to filter log entries so that only those categories specified in the filter are
consumed (see :ref:`dds_layer_log_filter`).
This component cannot be further configured or disabled.

.. _dds_layer_log_verbosity_level:

Verbosity Level
"""""""""""""""

*eProsima Fast DDS* logging module provides three verbosity levels defined by the |Log::Kind-api| enumeration,
those are:

* |Log::Kind::Error-api|: Used to log error messages.
* |Log::Kind::Warning-api|: Used to log error and warning messages.
* |Log::Kind::Info-api|: Used to log error, warning, and info messages.

The logging module's verbosity level defaults to |Log::Kind::Error-api|, which means that only messages logged with
|EPROSIMA_LOG_ERROR| would be consumed.
The verbosity level can be set and retrieved using member functions |Log::SetVerbosity-api| and |Log::GetVerbosity-api|
respectively.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_SET_GET_VERBOSITY
    :end-before: //!--
    :dedent: 4

.. warning::

    Setting any of the CMake options ``LOG_NO_INFO``, ``LOG_NO_WARNING`` or ``LOG_NO_ERROR`` to ``ON``
    will completely disable the corresponding verbosity level.
    ``LOG_NO_INFO`` is set to ``ON`` for Single-Config generators as default value if not in ``Debug`` mode.


.. _dds_layer_log_message:

Message
"""""""

This component constitutes the body of the log entry.
It is specified when producing the log via the macros presented in :ref:`dds_layer_log_logging`.
The message component can be used to filter log entries so that only those entries whose message pattern-matches the
filter are consumed (see :ref:`dds_layer_log_filter`).
This component cannot be further configured or disabled.


.. _dds_layer_log_file_context:

File Context
""""""""""""

This component specifies the origin of the log entry in terms of file name and line number (see
:ref:`dds_layer_log_logging` for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The file context component can be enabled/disabled using the member function |Log::ReportFilenames-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REPORT_FILENAMES
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_function_name:

Function Name
"""""""""""""

This component specifies the origin of the log entry in terms of the function name (see
:ref:`dds_layer_log_logging` for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The function name component can be enabled/disabled using the member function |Log::ReportFunctions-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REPORT_FUNCTIONS
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_register_consumers:

Register Consumers
^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* logging module supports zero or more :ref:`consumers <dds_layer_log_consumer>` logging the entries
registered in the logging queue with the methods described in :ref:`dds_layer_log_logging`.
To register a consumer, the |Log-api| class exposes member function |Log::RegisterConsumer-api|

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REGISTER_CONSUMER
    :end-before: //!--
    :dedent: 4

The consumers list can be emptied with member function |Log::ClearConsumers-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_CLEAR_CONSUMERS
    :end-before: //!--
    :dedent: 4

.. note::

    Registering and configuring consumers can also be done using *Fast DDS* XML configuration files.
    Please refer to :ref:`dds_layer_log_xml` for details.

.. warning::

    |Log::ClearConsumers-api| empties the consumers lists.
    All log entries are discarded until a new consumer is register via |Log::RegisterConsumer-api|, or until
    |Log::Reset-api| is called.

.. _dds_layer_log_reset:

Reset Configuration
^^^^^^^^^^^^^^^^^^^

The logging module's configuration can be reset to default settings with member function |Log::Reset-api|.

.. warning::

    Resetting the module's configuration entails:

    * Setting :ref:`dds_layer_log_verbosity_level` to |Log::Kind::Error-api|.
    * Disabling :ref:`dds_layer_log_file_context` component.
    * Enabling :ref:`dds_layer_log_function_name` component.
    * Clear all :ref:`dds_layer_log_filter`.
    * Clear all consumers and reset the default consumer according to CMake option |LOG_CONSUMER_DEFAULT|.


.. _dds_layer_log_xml:

XML Configuration
^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* allows for registering and configuring log consumers using XML configuration files.
Please refer to :ref:`Log profiles <logprofiles>` for details.
