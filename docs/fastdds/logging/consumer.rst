.. include:: includes/aliases.rst

.. _dds_layer_log_consumer:

Consumers
---------

Consumers are classes that take a |Log::Entry| and produce a log output accordingly.
*eProsima Fast DDS* provides two different log consumers that output log entries to different streams:

* :ref:`dds_layer_log_consumer_stdout`: Outputs log entries to STDOUT
* :ref:`dds_layer_log_consumer_file`: Outputs log entries to a user specified file.


.. _dds_layer_log_consumer_stdout:

StdoutConsumer
^^^^^^^^^^^^^^

|StdoutConsumer| is the default log consumer.
It outputs log entries to STDOUT stream following the convection specified in :ref:`dds_layer_log_logging_spec`.
By default, the logging module only has one consumer, which is a |StdoutConsumer|.
The |StdoutConsumer| can be registered and unregistered using the methods explained in
:ref:`dds_layer_log_register_consumers` and :ref:`dds_layer_log_reset`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_STDOUT_CONSUMER
    :end-before: //!--
    :dedent: 4


.. _dds_layer_log_consumer_file:

FileConsumer
^^^^^^^^^^^^

|FileConsumer| provides the logging module with log-to-file logging capabilities.
Applications willing to hold a persistent execution log record can specify a logging file using this consumer.
Furthermore, the application can choose whether the file stream should be in "write" or "append" mode, according to the
behaviour defined by |std::fstream::open|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_FILE_CONSUMER
    :end-before: //!--
    :dedent: 4
