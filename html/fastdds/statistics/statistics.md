<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics"></a>

<a id="statistics-module"></a>

# Statistics Module

The *Fast DDS Statistics module* is an extension of Fast DDS that enables the recollection of data concerning the DDS
communication.
The collected data is published using DDS over dedicated topics using builtin DataWriters within the
*Statistics module*.
In *Fast DDS*, the *Statistics module* and the [Monitor Service](monitor_service/monitor_service.md#monitor-service) are compiled when the `FASTDDS_STATISTICS`
CMake option is enabled, which is the default (see [CMake options](../../installation/configuration/cmake_options.md#cmake-options)).
For more information about *Fast DDS* compilation, see [Linux installation from sources](../../installation/sources/sources_linux.md#linux-sources) and [Windows installation from sources](../../installation/sources/sources_windows.md#windows-sources).

**[Pro]** In *Fast DDS* **[Pro]**, the *Statistics module* and the [Monitor Service](monitor_service/monitor_service.md#monitor-service) are always available, so no
specific build configuration is required to use them.

Data is only collected and published for the statistics topics that are explicitly enabled, meaning that applications
that do not enable any of them are not affected.
Therefore, the user must enable those DataWriters that are publishing data on the topics of interest for the user’s
application.
To that end, the standard [DDS Layer](../dds_layer/dds_layer.md#dds-layer) has been extended with the statistics API (see
[Statistics Domain Participant](dds_layer/domainParticipant.md#statistics-domainparticipant)).
The following section explains it.

#### NOTE
Please refer to [Statistics QoS Troubleshooting](dds_layer/troubleshooting.md#statistics-troubleshooting) for any problems related to the statistics module.

* [Statistics Module DDS Layer](dds_layer/statistics_dds_layer.md)
  * [Statistics Topic names](dds_layer/topic_names.md)
  * [Statistics Domain Participant](dds_layer/domainParticipant.md)
  * [Statistics recommended QoS](dds_layer/qos.md)
  * [Troubleshooting](dds_layer/troubleshooting.md)
* [Monitor Service](monitor_service/monitor_service.md)
  * [Introduction](monitor_service/intro.md)
  * [Monitor Service Topics](monitor_service/monitor_service_topics.md)
  * [Monitor Service Configuration](monitor_service/monitor_service_configuration.md)
