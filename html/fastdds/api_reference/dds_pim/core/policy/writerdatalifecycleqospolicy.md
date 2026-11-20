<a id="api-pim-writerdatalifecycleqospolicy"></a>

<a id="writerdatalifecycleqospolicy"></a>

# WriterDataLifecycleQosPolicy

### class WriterDataLifecycleQosPolicy

Specifies the behavior of the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) with regards to the lifecycle of the data-instances it manages. 

#### NOTE
Mutable Qos Policy 

#### WARNING
This Qos Policy will be implemented in future releases. 

### Public Functions

### inline WriterDataLifecycleQosPolicy()

Constructor. 

### inline virtual ~WriterDataLifecycleQosPolicy()

Destructor. 

### Public Members

### bool autodispose_unregistered_instances

Controls whether a [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

will automatically dispose instances each time they are unregistered. The setting autodispose_unregistered_instances = TRUE indicates that unregistered instances will also be considered disposed.

By default, true.
