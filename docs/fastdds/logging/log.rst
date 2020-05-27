.. include:: includes/aliases.rst

.. _dds_layer_log_log:

Log
===

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
