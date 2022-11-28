.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_logging:

Logging Messages
----------------

The logging of messages is handled by three dedicated macros, one for each available verbosity level (see
:ref:`dds_layer_log_verbosity_level`):

* |EPROSIMA_LOG_INFO|: Logs messages with |Log::Kind::Info-api| verbosity.
* |EPROSIMA_LOG_WARNING|: Logs messages with |Log::Kind::Warning-api| verbosity.
* |EPROSIMA_LOG_ERROR|: Logs messages with |Log::Kind::Error-api| verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module's configuration (see :ref:`dds_layer_log_logging_spec` and
:ref:`dds_layer_log_config_entry`).

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_MESSAGES
    :end-before: //!--
    :dedent: 4

There exist some old log macros used in previous versions: :code:`logInfo`, :code:`logWarning` and :code:`logError`.
These macros are still available as long as user does not manually disable them by :code:`ENABLE_OLD_LOG_MACROS`
CMake option or in-site macro :code:`ENABLE_OLD_LOG_MACROS_` before including *Log* module.
See section :ref:`old_log_macros_disable` for more information.

.. warning::

    Note that each message level is deactivated when CMake options ``LOG_NO_INFO``, ``LOG_NO_WARNING`` or
    ``LOG_NO_ERROR`` are set to ``ON`` respectively.
    For more information about how to enable and disable each individual logging macro, please refer to
    :ref:`dds_layer_log_disable`.
