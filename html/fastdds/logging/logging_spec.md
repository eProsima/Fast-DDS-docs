<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-logging-spec"></a>

<a id="log-entry-specification"></a>

# Log Entry Specification

Log entries created by [StdoutConsumer](consumer.md#dds-layer-ostream-consumer-stdout), [StdoutErrConsumer](consumer.md#dds-layer-ostream-consumer-stdouterr) and
[FileConsumer](consumer.md#dds-layer-ostream-consumer-file) (*eProsima Fast DDS* built-in [Consumers](consumer.md#dds-layer-log-consumer)) adhere to the
following structure:

```bash
<Timestamp> [<Category> <Verbosity Level>] <Message> (<File Name>:<Line Number>) -> Function <Function Name>
```

An example of such log entry is given by:

```bash
2020-05-27 11:45:47.447 [DOCUMENTATION_CATEGORY Error] This is an error message (example.cpp:50) -> Function main
```

#### NOTE
File Name and Line Number, as well as Function Name are only present when enabled. See
[Module Configuration](logging_configuration.md#dds-layer-log-config) for details.
