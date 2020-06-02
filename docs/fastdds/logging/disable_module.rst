.. include:: includes/aliases.rst

.. _dds_layer_log_disable:

Disable Logging Module
======================

Setting the :ref:`dds_layer_log_verbosity_level`, translates into entries not being added to the log queue if the
entry's level has lower importance than the set one.
This check is performed when calling the macros defined in :ref:`dds_layer_log_logging`.
However, it is possible to fully disable each macro (and therefore each verbosity level individually) at build time.

* |logInfo| is fully disabled by either:

    * Setting CMake option ``CMAKE_BUILD_TYPE`` to something other than ``Debug`` (``Release`` or ``RelWithDebInfo``).
    * Setting CMake option |LOG_NO_INFO| to ``ON``.
    * Defining macro |LOG_NO_INFO| to ``ON``

* |logWarning| is fully disabled by either:

    * Setting CMake option |LOG_NO_WARNING| to ``ON``.
    * Defining macro |LOG_NO_WARNING| to ``ON``

* |logError| is fully disabled by either:

    * Setting CMake option |LOG_NO_ERROR| to ``ON``.
    * Defining macro |LOG_NO_ERROR| to ``ON``

Applying either of the previously described methods will set the macro to be empty at configuration time, thus allowing
the compiler to optimize the call out.
|logInfo| is a special case worth mentioning; |logInfo| is only active is ``CMAKE_BUILD_TYPE`` is set to ``Debug``, or
if ``INTERNAL_DEBUG`` is set to ``ON``.
This is done so that all the debugging messages present on the library are optimized out at build time if not building
for debugging purposes, thus preventing them to impact performance.

.. warning::

    ``INTERNAL_DEBUG`` can be automatically set to ``ON`` if CMake option ``EPROSIMA_BUILD`` is set to ``ON``.
