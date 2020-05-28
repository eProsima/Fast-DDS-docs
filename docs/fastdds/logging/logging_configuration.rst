.. include:: includes/aliases.rst

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


.. _dds_layer_log_register_consumers:

Register Consumers
^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* logging module supports zero or :ref:`consumers <dds_layer_log_consumer>` logging the entries
registered in the logging queue with the methods described in :ref:`dds_layer_log_logging`.
To register a consumer, the |Log| class exposes member function |Log::RegisterConsumer|

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_REGISTER_CONSUMER
    :end-before: //!--
    :dedent: 4

The consumers list can be emptied with member function |Log::ClearConsumers|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_CLEAR_CONSUMERS
    :end-before: //!--
    :dedent: 4

.. note::

    Registering and configuring consumers can also be done using *Fast DDS* XML configuration files.
    Please refer to :ref:`dds_layer_log_xml` for details.

.. warning::

    |Log::ClearConsumers| empties the consumers lists.
    All log entries are discarded until a new consumer is register via |Log::RegisterConsumer|, or |Log::Reset| is
    called.

.. _dds_layer_log_reset:

Reset Configuration
^^^^^^^^^^^^^^^^^^^

The logging module's configuration can be reset to default settings with member function |Log::Reset|.

.. warning::

    Resetting the module's configuration entails:

    * Setting :ref:`dds_layer_log_verbosity_level` to |Log::Kind::Error|.
    * Disabling :ref:`dds_layer_log_file_context` component.
    * Enabling :ref:`dds_layer_log_function_name` component.
    * Clear all :ref:`dds_layer_log_filter`.
    * Clear all consumers and set a STDOUT consumer.


.. _dds_layer_log_xml:

XML Configuration
^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* allows for registering and configuring log consumers using XML configuration files.
This is done using the profile described in Log profiles.
Log profiles have the following structure.

- ``<use_default>``: If set to ``FALSE``, a call to |Log::ClearConsumers| is performed (see
  :ref:`dds_layer_log_register_consumers`).
- ``<consumer>``: Defines the class and configuration of the consumer to be registered.
  Multiple consumers can be registered this way.
  It has the following children elements:

  - ``<class>``: The class of the consumer.
    Possible values are ``StdoutConsumer`` and ``FileConsumer``.
  - ``<property>``: This element is used to configured the file consumer and therefore only applies if class is set to
    ``FileConsumer``. It has the following children elements:

    - ``<name>``: Name of the property to be configured.
      Possible values are ``filename`` and ``append``.
    - ``<value>``: The value of the property.
      Possible values adhere to the following rules:

      - If ``<name>`` is set to ``filename``, then this element contains the name of the log file.
      - If ``<name>`` is set to ``append``, then this element defined whether the consumer should, upon creation, open
        the file for appending or overriding.

The following constitutes an example of an XML configuration file that sets the log to use one StdoutConsumer and one
FileConsumer:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->LOG-CONFIG<-->
    :end-before: <!--><-->
    :lines: 2,3,5-32,34

.. note::

    For further details on writing and loading XML configuration files, please refer to :ref:`xml_profiles`.
