The *Fast DDS Statistics module* is an extension of Fast DDS that enables the recollection of data concerning the DDS
communication.
The collected data is published using DDS over dedicated topics using builtin DataWriters within the
*Statistics module*.
Consequently, by default, Fast DDS does not compile this module because it may entail affecting the application's
performance.
Nonetheless, the Statistics module can be activated using the ``-DFASTDDS_STATISTICS=ON`` at CMake configuration step.
For more information about *Fast DDS* compilation, see :ref:`linux_sources` and :ref:`windows_sources`.

Besides enabling the *Statistics Module* compilation, the user must enable those DataWriters that are publishing data on
the topics of interest for the user's application.
Therefore, the standard :ref:`dds_layer` has been extended.
The following section explains this DDS extended API.

.. note::
     It may happen that the default QoS settings for statistics DataWriters are not enough to handle deployments with
     several DomainParticipants with the *Statistics module* enabled resulting in statistics data loss. 

     Please refer to the |StatisticsQosTroubleshooting| section of the Statistics Recommended QoS page for more
     information on how to mitigate this issue.
