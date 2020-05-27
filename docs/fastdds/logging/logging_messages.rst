.. include:: includes/aliases.rst

.. _dds_layer_log_logging:

Logging messages
----------------

The logging of messages is handled by three dedicated macros, one for each available verbosity level:

* |logInfo|: Logs messages with |Log::Kind::Info| verbosity.
* |logWarning|: Logs messages with |Log::Kind::Warning| verbosity.
* |logError|: Logs messages with |Log::Kind::Error| verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module's configuration.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_MESSAGES
    :end-before: //!--
    :dedent: 4

Log entries adhere to the following structure:

.. code-block:: bash

    <Timestamp> [<Category> <Verbosity Level>] <Message> (<File Name>:<Line Number>) -> Function <Function Name>

An example of such log entry is given by:

.. code-block:: bash

    2020-05-27 11:45:47.447 [DOCUMENTATION_CATEGORY Error] This is an error message (example.cpp:50) -> Function main

.. note::

    `File Name` and `Line Number`, as well as `Function Name` are only present when enabled. See
    :ref:`dds_layer_log_config` for details.
