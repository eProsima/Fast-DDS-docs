.. include:: includes/aliases.rst

.. _dds_layer_log_intro:

Logging
=======

Fast DDS provides an extensible built-in logging module that exposes the following functionalities:

* Three different logging levels: |Log::Kind::Info|, |Log::Kind::Warning|, and |Log::Kind::Error|.
* Message filtering according to different criteria: category, content, or file of origin.
* Output to STDOUT and/or log file.

This section is devoted to explain the use, configuration, and extensibility of Fast DDS' :ref:`dds_layer_log_intro`.

.. figure:: /01-figures/fast_dds/log/class_diagram.svg
    :align: center

    Logging module class diagram

|Log| is the base class of the logging module. This singleton is not only in charge of the logging operations, but it
also provides configuration APIs to set different logging configuration aspects.

.. warning::

    |Log| public API exposes member function |Log::QueueLog|.
    However, this function is not intended to be used directly.
    To add messages to the log queue, use the methods described in :ref:`dds_layer_log_logging`.

.. toctree::

    /fastdds/logging/logging_messages
    /fastdds/logging/logging_configuration
    /fastdds/logging/logging_filtering
    /fastdds/logging/consumer
