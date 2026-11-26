<a id="api-pim-typeinformationparameter"></a>

<a id="typeinformationparameter"></a>

# TypeInformationParameter

### class TypeInformationParameter : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](../core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Class [xtypes::TypeInformationParameter](#classeprosima_1_1fastdds_1_1dds_1_1xtypes_1_1TypeInformationParameter)

### Public Functions

### inline TypeInformationParameter()

Constructor. 

### inline TypeInformationParameter(const [TypeInformationParameter](#_CPPv4N8eprosima7fastdds3dds6xtypes24TypeInformationParameter24TypeInformationParameterERK24TypeInformationParameter) &type)

Copy constructor. 

* **Parameters:**
  **type** – Another instance of [TypeInformationParameter](#classeprosima_1_1fastdds_1_1dds_1_1xtypes_1_1TypeInformationParameter)

### inline TypeInformationParameter(const eprosima::fastdds::dds::xtypes::TypeInformation &info)

Constructor using a TypeInformation. 

* **Parameters:**
  **info** – TypeInformation to be set 

### inline TypeInformationParameter([TypeInformationParameter](#_CPPv4N8eprosima7fastdds3dds6xtypes24TypeInformationParameter24TypeInformationParameterERR24TypeInformationParameter) &&type)

Move Constructor. 

* **Parameters:**
  **type** – Another instance of [TypeInformationParameter](#classeprosima_1_1fastdds_1_1dds_1_1xtypes_1_1TypeInformationParameter)

### virtual ~TypeInformationParameter() override = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](../core/policy/qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### inline bool assigned() const

Check if it is assigned. 

* **Returns:**
  true if assigned, false if not 

### inline void assigned(bool value)

Setter for assigned boolean. 

* **Parameters:**
  **value** – Boolean to be set 

### Public Members

### eprosima::fastdds::dds::xtypes::TypeInformation type_information

Type Information.
