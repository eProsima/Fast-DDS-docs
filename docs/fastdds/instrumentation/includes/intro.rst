The *Fast DDS Instrumentation module* is an extension to Fast DDS that enables the recollection of statistical data
concerning the DDS communication.
By default, Fast DDS does not compile this tool because it may entail affecting the application performance.
The Instrumentation module can be activating using the ``-DFASTDDS_INSTRUMENTATION=ON`` at CMake configuration step.
For more information about Fast DDS compilation, see :ref:`linux_sources` and :ref:`windows_sources`.

