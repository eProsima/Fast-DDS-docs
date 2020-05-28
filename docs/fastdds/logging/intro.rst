.. include:: includes/aliases.rst

.. _dds_layer_log_intro:

Logging
=======

*eProsima Fast DDS* provides an extensible built-in logging module that exposes the following functionalities:

* Three different logging levels: |Log::Kind::Info|, |Log::Kind::Warning|, and |Log::Kind::Error| (see
  :ref:`dds_layer_log_logging`).
* Message filtering according to different criteria: category, content, or file of origin (see
  :ref:`dds_layer_log_filter`).
* Output to STDOUT and/or log files (see :ref:`dds_layer_log_consumer`).

This section is devoted to explain the use, configuration, and extensibility of Fast DDS'
:ref:`Logging module <dds_layer_log_intro>`.

.. toctree::

    /fastdds/logging/logging_module
    /fastdds/logging/logging_spec
    /fastdds/logging/logging_thread
    /fastdds/logging/logging_messages
    /fastdds/logging/logging_configuration
    /fastdds/logging/logging_filtering
    /fastdds/logging/consumer
