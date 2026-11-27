<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-statistics-module-questions"></a>

<a id="statistics-module-frequently-asked-questions"></a>

# Statistics Module Frequently Asked Questions

**What is the purpose of the statistics module?**

<br/> The Fast DDS Statistics module is an extension of Fast DDS that enables the recollection of data concerning the DDS communication.
For further information, refer to [Statistics Module](../../statistics/statistics.md#statistics).

---

**How does the statistics module work?**

<br/> The collected data is published using DDS over dedicated topics using builtin DataWriters within the Statistics module.
For further information, refer to [Statistics Domain Participant](../../statistics/dds_layer/domainParticipant.md#statistics-domainparticipant).

---

**What is the consequence of compiling the statistics module?**

<br/> It may entail affecting the application’s performance.
For further information, refer to [Statistics Module](../../statistics/statistics.md#statistics).

---

**How can we activate the statistics module?**

<br/> It can be activated using the `-DFASTDDS_STATISTICS=ON` at CMake configuration step.
**[Pro]** In *Fast DDS* **[Pro]**, it is always compiled in, and it only collects data on the statistics topics enabled at
runtime.
For further information, refer to [Statistics Module](../../statistics/statistics.md#statistics).

---

**How can we start collecting data?**

<br/> In order to start collecting data in one of the statistics topics, the corresponding statistics DataWriter should be enabled.
For further information, refer to [Statistics Domain Participant](../../statistics/dds_layer/domainParticipant.md#statistics-domainparticipant).

---

**How can we enable the statistics DataWriter?**

<br/> It can either be done automatically or be enabled at run time using one of two methods: `enable_statistics_datawriter()` or `enable_statistics_datawriter_with_profile()`.
For further information, refer to [Enable statistics DataWriters](../../statistics/dds_layer/domainParticipant.md#statistics-enable-datawriters).

---

**Are all the statistics topics available in Fast DDS Pro?**

<br/> **[Pro]** In *Fast DDS* **[Pro]**, the `NETWORK_LATENCY_TOPIC`, `RTPS_SENT_TOPIC`, `RTPS_LOST_TOPIC`, `DISCOVERY_TOPIC`, `SAMPLE_DATAS_TOPIC` and `PHYSICAL_DATA_TOPIC` topics are no longer available.
Enabling any of them has no effect, and a warning is logged.
For further information, refer to [Statistics topics not available in Fast DDS](../../statistics/dds_layer/topic_names.md#statistics-topic-names-removed) .

---

**How is the environment variable "FASTDDS_STATISTICS" used?**

<br/> The environment variable is only used in the case where the CMake option `FASTDDS_STATISTICS` has been enabled
(always in *Fast DDS* **[Pro]**).
In any other case, the environment variable has no effect.
The statistics DataWriters that will be enabled when the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is enabled would be the union between those specified in the `properties()` `fastdds.statistics` and those included with the environment variable.
For further information, refer to [FASTDDS_STATISTICS](../../env_vars/env_vars.md#env-vars-fastdds-statistics).

---

**What is the monitor service?**

<br/> The Monitor Service targets any application implementing the subscription side of the Monitor Service Status Topic, giving the possibility of retrieving the Monitoring Information of the local entities.
For further information, refer to [Introduction](../../statistics/monitor_service/intro.md#monitor-intro).

---

**What Information can the monitor service carry?**

<br/> It can carry information about the monitoring information of the local entities of a particular DomainParticipant.
For further information, refer to [Introduction](../../statistics/monitor_service/intro.md#monitor-intro).

---

**How can the monitor service be activated?**

<br/> The Monitor Service can be activated using the `-DFASTDDS_STATISTICS=ON` at CMake configuration step.
**[Pro]** In *Fast DDS* **[Pro]**, it is always compiled in.
For further information, refer to [Monitor Service Configuration](../../statistics/monitor_service/monitor_service_configuration.md#monitor-service-configuration).

---

**At which layers can the monitor service be enabled?**

<br/> It can be programmatically enabled in both DDS Layer and RTPS Layer through the `enable_monitor_service()` and `disable_monitor_service()` calls.
For further information, refer to [Monitor Service Configuration](../../statistics/monitor_service/monitor_service_configuration.md#monitor-service-configuration).

<br/>
