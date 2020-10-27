.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_consumer:

Consumers
---------

Consumers are classes that take a |Log::Entry-api| and produce a log output accordingly.
*eProsima Fast DDS* provides three different log consumers that output log entries to different streams:

* :ref:`dds_layer_ostream_consumer_stdout`: Outputs log entries to STDOUT
* :ref:`dds_layer_ostream_consumer_stdouterr`: Outputs log entries to STDOUT or STDERR depending on the given threshold.
* :ref:`dds_layer_ostream_consumer_file`: Outputs log entries to a user specified file.


.. _dds_layer_ostream_consumer_stdout:

StdoutConsumer
^^^^^^^^^^^^^^

|StdoutConsumer-api| outputs log entries to STDOUT stream following the convection specified in
:ref:`dds_layer_log_logging_spec`.
It is the default and only log consumer of the logging module if the CMake option |LOG_CONSUMER_DEFAULT| is set to
``AUTO`` or ``STDOUT``.
It can be registered and unregistered using the methods explained in
:ref:`dds_layer_log_register_consumers` and :ref:`dds_layer_log_reset`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_STDOUT_CONSUMER
    :end-before: //!--
    :dedent: 4


.. _dds_layer_ostream_consumer_stdouterr:

StdoutErrConsumer
^^^^^^^^^^^^^^^^^

|StdoutErrConsumer-api| uses a |Log::Kind-api| threshold to filter the output of the log entries.
Those log entries whose |Log::Kind-api| is equal to or more severe than the given threshold output to STDERR.
Other log entries output to STDOUT.
By default, the threshold is set to |Log::Kind::Warning-api|.
|StdoutErrConsumer::stderr_threshold-api| allows the user to modify the default threshold.

Additionally, if CMake option |LOG_CONSUMER_DEFAULT| is set to ``STDOUTERR``, the logging module will use this consumer
as the default log consumer.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_STDOUTERR_CONSUMER
    :end-before: //!--
    :dedent: 4


.. _dds_layer_ostream_consumer_file:

FileConsumer
^^^^^^^^^^^^

|FileConsumer-api| provides the logging module with log-to-file logging capabilities.
Applications willing to hold a persistent execution log record can specify a logging file using this consumer.
Furthermore, the application can choose whether the file stream should be in "write" or "append" mode, according to the
behaviour defined by |std::fstream::open-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_FILE_CONSUMER
    :end-before: //!--
    :dedent: 4
