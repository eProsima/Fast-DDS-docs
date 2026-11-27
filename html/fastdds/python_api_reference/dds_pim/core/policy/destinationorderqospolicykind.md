<a id="python-api-pim-destinationorderqospolicykind"></a>

<a id="destinationorderqospolicykind"></a>

# DestinationOrderQosPolicyKind

### fastdds.BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS

Indicates that data is ordered based on the reception time at each Subscriber. Since each subscriber may receive
the data at different times there is no guaranteed that the changes will be seen in the same order. Consequently,
it is possible for each subscriber to end up with a different final value for the data.

### fastdds.BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS

Indicates that data is ordered based on a timestamp placed at the source (by the Service or by the application).
In any case this guarantees a consistent final value for the data in all subscribers.
