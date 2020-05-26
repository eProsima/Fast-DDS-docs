.. include:: includes/aliases.rst

.. _dds_layer_log_log:

Log
===

|Log| is the base class of the logging module. This singleton is not only in charge of the logging operations, but it
also provides configuration APIs to set different logging configuration aspects.


.. _dds_layer_log_logging:

Logging messages
----------------

The logging of messages is handled by three dedicated macros, one for each available verbosity level:

* |logInfo|: Logs messages with |Log::Kind::Info| verbosity.
* |logWarning|: Logs messages with |Log::Kind::Warning| verbosity.
* |logError|: Logs messages with |Log::Kind::Error| verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module's configuration.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_MESSAGES
    :end-before: //!--
    :dedent: 4

Log entries adhere to the following structure:

.. code-block:: bash

    <Timestamp> [<Category> <Verbosity Level>] <Message> (<file_name>:<line_number>) -> Function <function_name>

An example of such log entry is given by:

.. code-block:: bash

    2020-05-27 11:45:47.447 [DOCUMENTATION_CATEGORY Error] This is an error message (example.cpp:50) -> Function main

.. note::

    `file_name` and `line_number`, as well as `function_name` are only present when enabled. See
    :ref:`dds_layer_log_config` for details.


.. _dds_layer_log_config:

Logging module configuration
----------------------------

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
^^^^^^^^^

The log timestamp follows the ISO 8601 specification for local timestamps, i.e. *YYYY-MM-DD hh:mm:ss.sss*.
This component cannot be further configured or disabled.


.. _dds_layer_log_category:

Category
^^^^^^^^

Log entries have a category assigned when producing the log via the macros presented in
:ref:`dds_layer_log_logging`.
The category component can be used to filter log entries so that only those categories specified in the filter are
consumed (see :ref:`dds_layer_log_filter`).
This component cannot be further configured or disabled.

.. _dds_layer_log_verbosity_level:

Verbosity Level
^^^^^^^^^^^^^^^

*eProsima Fast DDS* logging module provides three verbosity levels defined by the |Log::Kind| enumeration, those are:

* |Log::Kind::Error|: Used to log error messages.
* |Log::Kind::Warning|: Used to log error and warning messages.
* |Log::Kind::Info|: Used to log error, warning, and info messages.

The logging module's verbosity level defaults to |Log::Kind::Error|, which means that only messages logged with
|logError| would be consumed.
The verbosity level can be set and retrieved using member functions |Log::SetVerbosity| and |Log::GetVerbosity|
respectively.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_SET_GET_VERBOSITY
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_message:

Message
^^^^^^^

This component constitutes the body of the log entry.
It is specified when producing the log via the macros presented in :ref:`dds_layer_log_logging`.
The message component can be used to filter log entries so that only those entries whose message pattern-matches the
filter are consumed (see :ref:`dds_layer_log_filter`).
This component cannot be further configured or disabled.


.. _dds_layer_log_file_context:

File Context
^^^^^^^^^^^^

This component specifies the origin of the log entry in terms of file name and line number (see
:ref:`dds_layer_log_logging` for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The file context component can be enabled/disabled using the member function |Log::ReportFilenames|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REPORT_FILENAMES
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_function_name:

Function Name
^^^^^^^^^^^^^

This component specifies the origin of the log entry in terms of the function name (see
:ref:`dds_layer_log_logging` for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The function name component can be enabled/disabled using the member function |Log::ReportFunctions|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REPORT_FUNCTIONS
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_filter:

Logging Filters
---------------
