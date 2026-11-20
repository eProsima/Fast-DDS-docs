<a id="python-api-pim-writerdatalifecycleqospolicy"></a>

<a id="writerdatalifecycleqospolicy"></a>

# WriterDataLifecycleQosPolicy

### *class* fastdds.WriterDataLifecycleQosPolicy

Specifies the behavior of the DataWriter with regards to the lifecycle of the data-instances it manages. Warning: This Qos Policy will be implemented in future releases. Notes: Mutable Qos Policy

#### *property* autodispose_unregistered_instances

Controls whether a DataWriter will automatically dispose instances each time they are unregistered.
The setting autodispose_unregistered_instances = TRUE indicates that unregistered instances will also be considered
disposed.

By default, true.

#### clear()

#### *property* thisown

The membership flag
