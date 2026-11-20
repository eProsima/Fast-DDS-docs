<a id="python-api-pim-ownershipstrengthqospolicy"></a>

<a id="ownershipstrengthqospolicy"></a>

# OwnershipStrengthQosPolicy

### *class* fastdds.OwnershipStrengthQosPolicy

Specifies the value of the “strength” used to arbitrate among multiple DataWriter objects that attempt to modify the same
instance of a data-object (identified by Topic + key).This policy only applies if the OWNERSHIP QoS policy is of kind
EXCLUSIVE.

Notes: Mutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* thisown

The membership flag

#### *property* value

Strength 
By default, 0.
