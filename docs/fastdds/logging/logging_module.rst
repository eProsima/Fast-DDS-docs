.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_classes:

Module Structure
----------------

The logging module provides the following classes:

* |Log-api| is the core class of the logging module.
  This singleton is not only in charge of the logging operations (see :ref:`dds_layer_log_logging`), but it also
  provides configuration APIs to set different logging configuration aspects (see :ref:`dds_layer_log_config`), as well
  as logging filtering at various levels (see :ref:`dds_layer_log_filter`).
  It contains zero or more |LogConsumer-api| objects.
  The singleton's consuming thread feeds the log entries added to the logging queue using the macros defined in
  :ref:`dds_layer_log_logging` to the log consumers sequentially (see :ref:`dds_layer_log_thread`).

  .. warning::

      |Log-api| API exposes member function |Log::QueueLog-api|.
      However, this function is not intended to be used directly.
      To add messages to the log queue, use the methods described in :ref:`dds_layer_log_logging`.

* |LogConsumer-api| is the base class for all the log consumers (see :ref:`dds_layer_log_consumer`).
  It includes the member functions that derived classes should overload to consume log entries.

  - |OstreamConsumer-api| derives from |LogConsumer-api|.
    It defines how to consume log entries for outputting to an |std::ostream-api| object.
    It includes a member function that derived classes must overload to define the desired |std::ostream-api| object.

    1. |StdoutConsumer-api| derives from |OStreamConsumer-api|.
    It defines STDOUT as the output |std::ostream-api| object (see :ref:`dds_layer_ostream_consumer_stdout`).

    2. |StdoutErrConsumer-api| derives from |OStreamConsumer-api|.
    It defines a |Log::Kind-api| threshold so that if the |Log::Kind-api| is equal to or more severe than the selected
    threshold, the output defined will be STDERR.
    Otherwise, it defines STDOUT as the output (see :ref:`dds_layer_ostream_consumer_stdouterr`).

    3. |FileConsumer-api| derives from |OStreamConsumer-api|.
    It defines an user specified file as the output |std::ostream-api| object
    (see :ref:`dds_layer_ostream_consumer_file`).

.. figure:: /01-figures/fast_dds/log/class_diagram.svg
    :align: center

    Logging module class diagram

The module can be further extended by creating new consumer classes deriving from |LogConsumer-api| and/or
|OStreamConsumer-api|.
To enable a custom consumer just follow the instructions on :ref:`dds_layer_log_register_consumers`.
