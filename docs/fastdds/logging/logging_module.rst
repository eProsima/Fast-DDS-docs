.. include:: includes/aliases.rst

.. _dds_layer_log_classes:

Module Structure
----------------

The logging module provides the following classes:

* |Log| is the core class of the logging module.
  This singleton is not only in charge of the logging operations (see :ref:`dds_layer_log_logging`), but it also
  provides configuration APIs to set different logging configuration aspects (see :ref:`dds_layer_log_config`), as well
  as logging filtering at various levels (see :ref:`dds_layer_log_filter`).
  It contains zero or more |LogConsumer| objects.
  The singleton's consuming thread feeds the log entries added to the logging queue using the macros defined in
  :ref:`dds_layer_log_logging` to the log consumers sequentially (see :ref:`dds_layer_log_thread`).

  .. warning::

      |Log| API exposes member function |Log::QueueLog|.
      However, this function is not intended to be used directly.
      To add messages to the log queue, use the methods described in :ref:`dds_layer_log_logging`.

* |LogConsumer| is the base class for all the log consumers (see :ref:`dds_layer_log_consumer`).
  Includes the member functions that derived classes should overload to consume log entries.

  - |StdoutConsumer| derives from |LogConsumer|.
    It defines how to consume log entries for outputting to STDOUT (see :ref:`dds_layer_log_consumer_stdout`).
  - |FileConsumer| derives from |LogConsumer|.
    It defines how to consume log entries for outputting to a user specified file (see
    :ref:`dds_layer_log_consumer_file`).

.. figure:: /01-figures/fast_dds/log/class_diagram.svg
    :align: center

    Logging module class diagram

The module can be further extended by creating new consumer classes deriving from |LogConsumer|.
To enable a custom consumer just follow the instructions on :ref:`dds_layer_log_register_consumers`.
