The *Fast DDS Statistics module* is an extension to Fast DDS that enables the recollection of data
concerning the DDS communication.
The collected data is published using DDS over some specific dedicated topics using builtin DataWriters within
the *Statistics module*.
Consequently, by default, Fast DDS does not compile this module because it may entail affecting the application
performance.
The Statistics module can be activated using the ``-DFASTDDS_STATISTICS=ON`` at CMake configuration step.
For more information about Fast DDS compilation, see :ref:`linux_sources` and :ref:`windows_sources`.

Besides enabling the *Statistics Module* compilation, the user must enable those DataWriters that are
publishing data on those topics that may be of interest for the user's application.
Therefore, the standard :ref:`dds_layer` has been extended.

The following section explains this DDS extended API.
