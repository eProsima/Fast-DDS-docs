<a id="python-api-pim-presentationqospolicy"></a>

<a id="presentationqospolicy"></a>

# PresentationQosPolicy

### *class* fastdds.PresentationQosPolicy

Specifies how the samples representing changes to data instances are presented to the subscribing application.
This policy affects the application’s ability to specify and receive coherent changes and to see the relative
order of changes.access_scope determines the largest scope spanning the entities for which the order and coherency
of changes can be preserved. The two booleans control whether coherent access and ordered access are supported within
the scope access_scope.

Warning: This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.

Notes: Immutable Qos Policy

#### *property* access_scope

Access Scope Kind 
: By default, INSTANCE_PRESENTATION_QOS.

#### clear()

Clears the QosPolicy object

#### *property* coherent_access

Specifies support coherent access. That is, the ability to group a set of changes as a unit
on the publishing end such that they are received as a unit at the subscribing end.
by default, false.

#### *property* ordered_access

Specifies support for ordered access to the samples received at the subscription end. That is,
the ability of the subscriber to see changes in the same order as they occurred on the publishing end.
By default, false.

#### *property* thisown

The membership flag
