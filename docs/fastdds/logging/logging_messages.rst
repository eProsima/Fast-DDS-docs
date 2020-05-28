.. include:: includes/aliases.rst

.. _dds_layer_log_logging:

Logging messages
----------------

The logging of messages is handled by three dedicated macros, one for each available verbosity level (see
:ref:`dds_layer_log_verbosity_level`):

* |logInfo|: Logs messages with |Log::Kind::Info| verbosity.
* |logWarning|: Logs messages with |Log::Kind::Warning| verbosity.
* |logError|: Logs messages with |Log::Kind::Error| verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module's configuration (see :ref:`dds_layer_log_logging_spec` and
:ref:`dds_layer_log_config`).

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_MESSAGES
    :end-before: //!--
    :dedent: 4

.. warning::

    Beware that |logInfo| is deactivated  when compiled with ``CMAKE_BUILD_TYPE`` other than ``Debug``. More more
    information about how to enable and disable each individual logging macro, please refer to
    :ref:`dds_layer_log_disable`.
