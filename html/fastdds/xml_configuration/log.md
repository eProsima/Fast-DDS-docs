<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="logprofiles"></a>

<a id="log-profiles"></a>

# Log profiles

*eProsima Fast DDS* allows for registering and configuring [Log consumers](../logging/consumer.md#dds-layer-log-consumer) using XML
configuration files.
Please refer to [Logging](../logging/intro.md#dds-layer-log-intro) for more information on *Fast DDS* extensible Logging built-in module.
The logging profiles are defined within the `<log>` XML tags.
The `<log>` element has two child elements: `<use_default>` and `<consumer>`.
These are described in the following table.

| Name                | Description                                                                                                                                                                                                                                                                                                                                                                           | Values                                                                             | Default   |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------|-----------|
| `<use_default>`     | If set to `FALSE`, a call to [`Log::ClearConsumers()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log14ClearConsumersEv) is<br/>[`Log::ClearConsumers()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log14ClearConsumersEv) is<br/>performed. See [Register Consumers](../logging/logging_configuration.md#dds-layer-log-register-consumers). | `bool`                                                                             | `true`    |
| `<consumer>`        | Defines the class and configuration of the consumer to<br/>be registered. Multiple consumers can be registered<br/>this way. See [Consumers](../logging/consumer.md#dds-layer-log-consumer).                                                                                                                                                                                          | [ConsumerDataType](#xmllogconsumer)                                                |           |
| `<thread_settings>` | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the logging thread.                                                                                                                                                                                                                                                                            | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           |

The following constitutes an example of an XML configuration file that sets the [`Log`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE) to use one
[`StdoutConsumer`](../api_reference/log/stdout_consumer.md#_CPPv4N8eprosima7fastdds3dds14StdoutConsumerE), one [`StdoutErrConsumer`](../api_reference/log/stdouterr_consumer.md#_CPPv4N8eprosima7fastdds3dds17StdoutErrConsumerE), and one [`FileConsumer`](../api_reference/log/file_consumer.md#_CPPv4N8eprosima7fastdds3dds12FileConsumerE):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <log>
        <!--
        Clear consumers
        -->
        <use_default>false</use_default>

        <!--
        StdoutConsumer does not have any properties
        -->
        <consumer>
            <class>StdoutConsumer</class>
        </consumer>

        <!--
        StdoutErrConsumer with threshold set to Log::Kind::Error
        -->
        <consumer>
            <class>StdoutErrConsumer</class>
            <property>
                <name>stderr_threshold</name>
                <value>Log::Kind::Error</value>
            </property>
        </consumer>

        <!--
        FileConsumer openning "execution.log" in append mode
        -->
        <consumer>
            <class>FileConsumer</class>
            <property>
                <name>filename</name>
                <value>execution.log</value>
            </property>
            <property>
                <name>append</name>
                <value>true</value>
            </property>
        </consumer>

        <!--
        ThreadSettings for the log's consumer thread
        -->
        <thread_settings>
            <scheduling_policy>-1</scheduling_policy>
            <priority>0</priority>
            <affinity>0</affinity>
            <stack_size>-1</stack_size>
        </thread_settings>
    </log>
</dds>
```

<a id="xmllogconsumer"></a>

<a id="consumerdatatype"></a>

## ConsumerDataType

| Name                | Description                                                                                                                           | Values                      |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------|-----------------------------|
| `<class>`           | The class of the consumer.                                                                                                            | `StdoutConsumer`            |
| `StdoutErrConsumer` |                                                                                                                                       |                             |
| `FileConsumer`      |                                                                                                                                       |                             |
| `<property>`        | This element is used to configure the log consumer and only applies<br/>if `<class>` is set to `StdoutErrConsumer` or `FileConsumer`. | [PropertyType](#xmllogprop) |

<a id="xmllogprop"></a>

<a id="propertytype"></a>

## PropertyType

| Name                                                                                                                                                                                                                                                          | Description                            | Values               | Default   |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------|----------------------|-----------|
| `<name>`                                                                                                                                                                                                                                                      | Name of the property to be configured. | `filename`           |           |
| `append`                                                                                                                                                                                                                                                      |                                        |                      |           |
| `stderr_threshold`                                                                                                                                                                                                                                            |                                        |                      |           |
| `<value>`                                                                                                                                                                                                                                                     | The value of the property.             |                      |           |
| * If `<name>` is set to `filename`, then this<br/>  element contains the name of the log file. This<br/>  property only applies if `<class>` is set to<br/>  `FileConsumer`                                                                                   | `string`                               | output.log           |           |
| * If `<name>` is set to `append`, then this<br/>  element defines whether the consumer should, upon<br/>  creation, open the file for appending or<br/>  overriding. This property only applies if<br/>  `<class>` is set to `FileConsumer`                   | `Boolean`                              | `false`              |           |
| * If `<name>` is set to `stderr_threshold`, then<br/>  this element defines the threshold used by the<br/>  [Log consumers](../logging/consumer.md#dds-layer-log-consumer).<br/>  This property only applies if `<class>` is set<br/>  to `StdoutErrConsumer` | `Log::Kind`                            | `Log::Kind::Warning` |           |
