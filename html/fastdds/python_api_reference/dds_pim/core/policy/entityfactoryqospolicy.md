<a id="python-api-pim-entityfactoryqospolicy"></a>

<a id="entityfactoryqospolicy"></a>

# EntityFactoryQosPolicy

### *class* fastdds.EntityFactoryQosPolicy(\*args)

Controls the behavior of the entity when acting as a factory for other entities. In other words,
configures the side-effects of the create_\* and delete_\* operations.

Notes: Mutable Qos Policy

#### *property* autoenable_created_entities

Specifies whether the entity acting as a factory automatically enables the instances it creates.
If True the factory will automatically enable each created Entity otherwise it will not.

By default, True.

#### clear()

#### *property* thisown

The membership flag
