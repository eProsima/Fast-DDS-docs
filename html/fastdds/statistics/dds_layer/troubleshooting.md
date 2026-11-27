<a id="statistics-troubleshooting"></a>

<a id="troubleshooting"></a>

# Troubleshooting

This section aims to give quick solutions to overcome the most common problems arising from the use of the statistics
module.

<a id="statistics-troubleshooting-notgettindata"></a>

<a id="monitoring-application-is-not-receiving-any-statistic-data"></a>

## Monitoring application is not receiving any statistic data

Sometimes, especially in the case of monitoring large applications with many DataWriters and DataReaders, it may happen
that the application monitoring Fast DDS statistics does not receive any data.
This is generally caused by the default configuration of the statistics DataWriters, which includes the
`push_mode` set to `false` (i.e. `pull_mode`), the History Kind set to `KEEP_LAST`, and the
History Depth set to `10`.
With this configuration, the following may happen:

1. Fast DDS adds new samples to one of the statistics DataWriters.
2. The DataWriter notifies the DataReader of the availability of said samples.
3. The DataReader sends a request to the DataWriter to “pull” those samples.
4. Before the request arrives to the DataWriter, some new statistics samples are added to that same DataWriter, which
   causes the previous samples to be overwritten.
5. Once the DataReader request arrives to the DataWriter, since the requested samples have been overwritten, they are
   not available any more, so the DataWriter send a notification to the DataReader informing of the presence of the
   newer samples instead.
6. The loop starts again.

The easiest fix to overcome this situation is to simply increase the History Depth of the DataWriter to create Some
buffer to answer to requests:

### Generic profile

```xml
<?xml version="1.0" encoding="utf-8"?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="statistics_domainparticipant_conf_xml_general_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <!-- Activate various Fast DDS Statistics Module DataWriters -->
                        <property>
                            <name>fastdds.statistics</name>
                            <value>HISTORY_LATENCY_TOPIC;PDP_PACKETS_TOPIC</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>

        <!-- Generic profile for all the statistics DataWriter -->
        <data_writer profile_name="GENERIC_STATISTICS_PROFILE">
            <!-- Configure History QoS as KEEP_LAST 20 -->
            <!-- History depth depends on the user application constraints (publication rate for instance) -->
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
            </topic>
            <!-- Enable pull mode -->
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>fastdds.push_mode</name>
                        <value>false</value>
                    </property>
                </properties>
            </propertiesPolicy>
            <!-- Set durability, reliability, and publication mode -->
            <qos>
                <durability>
                    <kind>TRANSIENT_LOCAL</kind>
                </durability>

                <reliability>
                    <kind>RELIABLE</kind>
                </reliability>

                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </data_writer>
    </profiles>
</dds>
```

### Specific profile

```xml
<?xml version="1.0" encoding="utf-8"?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="statistics_domainparticipant_conf_xml_specific_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <!-- Activate various Fast DDS Statistics Module DataWriters -->
                        <property>
                            <name>fastdds.statistics</name>
                            <value>HISTORY_LATENCY_TOPIC</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>

        <!-- Generic profile for a specific statistics DataWriters -->
        <data_writer profile_name="HISTORY_LATENCY_TOPIC">
            <!-- Configure History QoS as KEEP_LAST 20 -->
            <!-- History depth depends on the user application constraints (publication rate for instance) -->
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
            </topic>
            <!-- Enable pull mode -->
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>fastdds.push_mode</name>
                        <value>false</value>
                    </property>
                </properties>
            </propertiesPolicy>
            <!-- Set durability, reliability, and publication mode -->
            <qos>
                <durability>
                    <kind>TRANSIENT_LOCAL</kind>
                </durability>

                <reliability>
                    <kind>RELIABLE</kind>
                </reliability>

                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </data_writer>
    </profiles>
</dds>
```

#### WARNING
Mind that the specific profile only targets to the DataWriter which profile name matches with the alias of
the corresponding statistics topic.
Check the complete list of topics and their aliases in the [Statistics Topic names](topic_names.md#statistics-topic-names) section.

#### NOTE
Increasing the History Depth of the statistics DataWriters has an impact on memory usage, as sufficient space is
pre-allocated for each of the DataWriter’s histories to hold that number of samples per topic instance.
