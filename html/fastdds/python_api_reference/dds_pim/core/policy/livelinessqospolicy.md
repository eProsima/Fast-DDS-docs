<a id="python-api-pim-livelinessqospolicy"></a>

<a id="livelinessqospolicy"></a>

# LivelinessQosPolicy

### *class* fastdds.LivelinessQosPolicy

Determines the mechanism and parameters used by the application to determine whether an Entity is “active” (alive).
The “liveliness” status of an Entity is used to maintain instance ownership in combination with the setting of the
OwnershipQosPolicy.
The application is also informed via listener when an Entity is no longer alive.

The DataReader requests that liveliness of the writers is maintained by the requested means and loss of liveliness is
detected with delay not to exceed the lease_duration.

The DataWriter commits to signaling its liveliness using the stated means at intervals not to exceed the lease_duration.
Listeners are used to notify the DataReaderof loss of liveliness and DataWriter of violations to the liveliness contract.

#### *property* announcement_period

> The period for automatic assertion of liveliness.
> Only used for DataWriters with AUTOMATIC liveliness.
> By default, dds::c_TimeInfinite.

Warning: When not infinite, must be < lease_duration, and it is advisable to be less than 0.7\*lease_duration.

#### clear()

Clears the QosPolicy object

#### *property* kind

Liveliness kind 
By default, AUTOMATIC_LIVELINESS.

#### *property* lease_duration

Period within which liveliness should be asserted.
On a DataWriter it represents the period it commits to signal its liveliness.
On a DataReader it represents the period without assertion after which a DataWriter is considered
inactive.
By default, dds::c_TimeInfinite.

#### *property* thisown

The membership flag
