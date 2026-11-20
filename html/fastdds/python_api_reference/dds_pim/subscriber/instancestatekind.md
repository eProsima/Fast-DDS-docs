<a id="python-api-pim-instancestatekind"></a>

<a id="instancestatekind"></a>

# InstanceStateKind

### fastdds.ALIVE_INSTANCE_STATE

Instance is currently in existence.

### fastdds.NOT_ALIVE_DISPOSED_INSTANCE_STATE

Not alive disposed instance. The instance has been disposed by a DataWriter.

### fastdds.NOT_ALIVE_NO_WRITERS_INSTANCE_STATE

Not alive no writers for instance. None of the ‘DataWriter’ objects that are
currently alive (according to the LIVELINESS QoS) are writing the instance.
