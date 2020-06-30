.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_thread:

Logging Thread
^^^^^^^^^^^^^^

Calls to the macros presented in :ref:`dds_layer_log_logging` merely add the log entry to a ready-to-consume queue.
Upon creation, the logging module spawns a thread that awakes every time an entry is added to the queue.
When awaken, this thread feeds all the entries in the queue to all the registered :ref:`dds_layer_log_consumer`.
Once the work is done, the thread falls back into idle state.
This strategy prevents the module from blocking the application thread when a logging operation is performed.
However, sometimes applications may want to wait until the logging routine is done to continue their operation.
The logging module provides this capability via the member function |Log::Flush-api|.
Furthermore, it is possible to completely eliminate the thread and its resources using member function
|Log::KillThread-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_FLUSH_AND_KILL
    :end-before: //!--
    :dedent: 4

.. warning::

    A call to any of the macros present in :ref:`dds_layer_log_logging` will spawn the logging thread even if it has
    been previously killed with |Log::KillThread-api|.
