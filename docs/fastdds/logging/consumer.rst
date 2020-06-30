.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_consumer:

Consumers
---------

Consumers are classes that take a |Log::Entry-api| and produce a log output accordingly.
*eProsima Fast DDS* provides two different log consumers that output log entries to different streams:

* :ref:`dds_layer_log_consumer_stdout`: Outputs log entries to STDOUT
* :ref:`dds_layer_log_consumer_file`: Outputs log entries to a user specified file.


.. _dds_layer_log_consumer_stdout:

StdoutConsumer
^^^^^^^^^^^^^^

|StdoutConsumer-api| is the default log consumer.
It outputs log entries to STDOUT stream following the convection specified in :ref:`dds_layer_log_logging_spec`.
By default, the logging module only has the StdoutConsumer.
It can be registered and unregistered using the methods explained in
:ref:`dds_layer_log_register_consumers` and :ref:`dds_layer_log_reset`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_STDOUT_CONSUMER
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_consumer_file:

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
