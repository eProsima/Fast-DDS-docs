<a id="python-api-pim-destinationorderqospolicy"></a>

<a id="destinationorderqospolicy"></a>

# DestinationOrderQosPolicy

### *class* fastdds.DestinationOrderQosPolicy

Controls the criteria used to determine the logical order among changes made by Publisher entities to the same instance of
data (i.e., matching Topic and key).

Warning: This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* kind

DestinationOrderQosPolicyKind. 
: By default, BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS.

#### *property* thisown

The membership flag
