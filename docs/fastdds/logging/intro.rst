.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_intro:

Logging
=======

*eProsima Fast DDS* provides an extensible built-in logging module that exposes the following main functionalities:

* Three different logging levels: |Log::Kind::Info-api|, |Log::Kind::Warning-api|, and |Log::Kind::Error-api| (see
  :ref:`dds_layer_log_logging`).
* Message filtering according to different criteria: category, content, or source file (see
  :ref:`dds_layer_log_filter`).
* Output to STDOUT, STDERR and/or log files (see :ref:`dds_layer_log_consumer`).

This section is devoted to explain the use, configuration, and extensibility of Fast DDS' logging module.

.. toctree::

    /fastdds/logging/logging_module
    /fastdds/logging/logging_spec
    /fastdds/logging/logging_thread
    /fastdds/logging/logging_messages
    /fastdds/logging/logging_configuration
    /fastdds/logging/logging_filtering
    /fastdds/logging/consumer
    /fastdds/logging/disable_module
