<a id="python-api-pim-incompatibleqosstatus"></a>

<a id="incompatibleqosstatus"></a>

# IncompatibleQosStatus

### *class* fastdds.IncompatibleQosStatus

A struct storing the requested incompatible QoS status

#### *property* last_policy_id

The id of the policy that was found to be incompatible the last time an incompatibility is detected

#### *property* policies

A list of QosPolicyCount

#### *property* thisown

The membership flag

#### *property* total_count

Total cumulative number of times the concerned writer discovered a reader for the same topic The requested QoS is incompatible with the one offered by the writer

#### *property* total_count_change

The change in total_count since the last time the listener was called or the status was read
