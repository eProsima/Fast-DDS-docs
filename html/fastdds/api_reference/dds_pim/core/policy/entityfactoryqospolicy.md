<a id="api-pim-entityfactoryqospolicy"></a>

<a id="entityfactoryqospolicy"></a>

# EntityFactoryQosPolicy

### class EntityFactoryQosPolicy

Controls the behavior of the entity when acting as a factory for other entities. In other words, configures the side-effects of the create_\* and delete_\* operations. 

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline EntityFactoryQosPolicy()

Constructor without parameters. 

### inline EntityFactoryQosPolicy(bool autoenable)

Constructor. 

* **Parameters:**
  **autoenable** – Value for the autoenable_created_entities boolean 

### inline virtual ~EntityFactoryQosPolicy()

Destructor. 

### Public Members

### bool autoenable_created_entities

Specifies whether the entity acting as a factory automatically enables the instances it creates. If True the factory will automatically enable each created [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity)

otherwise it will not.

By default, True.
