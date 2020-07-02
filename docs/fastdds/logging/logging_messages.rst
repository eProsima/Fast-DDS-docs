.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_logging:

Logging Messages
----------------

The logging of messages is handled by three dedicated macros, one for each available verbosity level (see
:ref:`dds_layer_log_verbosity_level`):

* |logInfo|: Logs messages with |Log::Kind::Info-api| verbosity.
* |logWarning|: Logs messages with |Log::Kind::Warning-api| verbosity.
* |logError|: Logs messages with |Log::Kind::Error-api| verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module's configuration (see :ref:`dds_layer_log_logging_spec` and
:ref:`dds_layer_log_config_entry`).

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_MESSAGES
    :end-before: //!--
    :dedent: 4

.. warning::

    Note that |logInfo| is deactivated  when compiled with ``CMAKE_BUILD_TYPE`` other than ``Debug``. More more
    information about how to enable and disable each individual logging macro, please refer to
    :ref:`dds_layer_log_disable`.
