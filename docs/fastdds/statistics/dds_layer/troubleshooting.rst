.. _statistics_troubleshooting:

Troubleshooting
===============

This section aims to give quick solutions to overcome the most common problems arising from the use of the statistics
module.

.. _statistics_troubleshooting_notgettindata:

Monitoring application is not receiving any statistic data
----------------------------------------------------------

Sometimes, especially in the case of monitoring large applications with many DataWriters and DataReaders, it may happen
that the application monitoring Fast DDS statistics does not receive any data.
This is generally caused by the default configuration of the statistics DataWriters, which includes the
:class:`push_mode` set to :class:`false` (i.e. :class:`pull_mode`), the History Kind set to :class:`KEEP_LAST`, and the
History Depth set to :class:`10`.
With this configuration, the following may happen:

1. Fast DDS adds new samples to one of the statistics DataWriters.
2. The DataWriter notifies the DataReader of the availability of said samples.
3. The DataReader sends a request to the DataWriter to "pull" those samples.
4. Before the request arrives to the DataWriter, some new statistics samples are added to that same DataWriter, which
   causes the previous samples to be overwritten.
5. Once the DataReader request arrives to the DataWriter, since the requested samples have been overwritten, they are
   not available any more, so the DataWriter send a notification to the DataReader informing of the presence of the
   newer samples instead.
6. The loop starts again.

The easiest fix to overcome this situation is to simply increase the History Depth of the DataWriter to create Some
buffer to answer to requests:

.. tabs::

  .. tab:: Generic profile

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->FASTDDS_STATISTICS_TROUBLESHOOTING_GENERIC<-->
            :end-before: <!--><-->
            :lines: 2-4, 6-53, 55-56

  .. tab:: Specific profile

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->FASTDDS_STATISTICS_TROUBLESHOOTING_SINGLE_TOPIC<-->
            :end-before: <!--><-->
            :lines: 2-4, 6-53, 55-56

        .. warning::

            Mind that the specific profile only targets to the DataWriter which profile name matches with the alias of
            the corresponding statistics topic.
            Check the complete list of topics and their aliases in the :ref:`statistics_topic_names` section.

.. note::
    Increasing the History Depth of the statistics DataWriters has an impact on memory usage, as sufficient space is
    pre-allocated for each of the DataWriter's histories to hold that number of samples per topic instance.
