.. _dynamic-types:

Dynamic Topic Types
===================

.. _link: http://www.omg.org/spec/DDS-XTypes/1.2

eProsima Fast-RTPS provides a dynamic way to define and use topic types and topic data.
Our implementation follows the OMG Extensible and Dynamic Topic Types for DDS interface.
For more information you can read the document (DDS-XTypes V1.2) in this link_.

The dynamic topic types offers the possibility to work over rtps without the restrictions related with the IDLs.
Using them the users can declare the different types that they need and manage the information directly,
avoiding the additional step of updating the IDL file and the generation of C++ classes.

The management of dynamic types is splitted into two main groups.
The first one manages the declaration of the types, building and
setting the configuration of every type and the second one is in charge of
the data instances and their information.

**Type Descriptor**

Stores the information about one type with its relationships and restrictions.
It's the minimum class needed to generate a Dynamic type, and in case of the
complex ones, it stores information about its children or its parent types.

**Member Descriptor**

Several complex types need member descriptors to declare the relationship between types.
This class stores information about that members like their name, their unique ID,
the type that is going to be created and the default value after the creation.
Union types have special fields to identify each member by labels.

**Dynamic Type Builder Factory**

Singleton class that is in charge of the creation and the management of every
Dynamic Types and Dynamic Type Builders.
It declares methods to create each kind of supported types, making easier the
management of the descriptors.
Every object created by the factory must be deleted calling the DeleteType method.

**Dynamic Type Builder**

Intermediate class used to configure and create Dynamic Types.
By design Dynamic types can't be modified, so the previous step to create a
new one is to create a builder and apply the settings that the user needs.
Users can create several types using the same builder, but the changes applied
to the builder don't affect to the types created previously.
Every object created by a builder must be deleted calling the DeleteType method
of the Dynamic Type builder Factory.

**Dynamic Type**

Base class in the declaration of Dynamic types, it stores the information about
its type and every Member that is related to it.
It creates a copy of the descriptor on its creation and cannot be changed to keep the consistency.

**Dynamic Type Member**

Class that creates the relationship between a member descriptor with its parent type.
Dynamic Types have a one Dynamic type member for every child member added to it.

**Dynamic Data Factory**

Singleton class that is in charge of the creation and the management of every
DynamicData.
It creates them using the given Dynamic Type with its settings.
Every data object created by the factory must be deleted calling the DeleteType method.
Allows to create a TypeIdentifier and a (Minimal)TypeObject from a TypeDescriptor.
CompleteTypeObject support is planned to be added in the future.

**Dynamic Data**

Class that manages the data of the Dynamic Types. It stores the information that is
sent and received.
There are two ways to work with DynamicDatas, the first one is the
most secured, activating the macro `DYNAMIC_TYPES_CHECKING`, it creates a variable for
each primitive kind to help the debug process.
The second one reduces the size of the DynamicData class using only the minimum
values and making the code harder to debug.

**Dynamic PubSubType**

Class that inherits from TopicDataType and works as an intermediator between RTPS
Domain and the Dynamic Types. It implements the methods needed to create, serialize,
deserialize and delete dynamicdatas when the participants need to convert the
received information from any transport to the registered dynamic type.


Supported Types
---------------

Primitive Types
^^^^^^^^^^^^^^^

This section includes every simple kinds:

- BYTE
- CHAR8
- CHAR16
- BOOLEAN
- INT16
- UINT16
- INT32
- UINT32
- INT64
- UINT64
- FLOAT32
- FLOAT64
- FLOAT128

Primitive types don't need an specific configuration to create the type. Because of that
DynamicTypeBuilderFactory has got exposed several methods to allow users to create
the Dynamic Types avoiding the DynamicTypeBuilder step. The example below shows the two
ways to create a dynamic data of a primitive type.
The DynamicData class has a specific `Get` and `Set` Methods for each primitive
type of the list.

.. code-block:: c++

    // Using Builders
    DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
    DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(created_type);
    data->SetInt32Value(1);

    // Creating directly the Dynamic Type
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
    data2->SetInt32Value(1);

String and WString
^^^^^^^^^^^^^^^^^^

Strings are pretty similar to primitive types with one exception, they need to set the size
of the buffer that they can manage.
To do that, DynamicTypeBuilderFactory exposes the methods `CreateStringType` and `CreateWstringType`.
By default its size is set to 255 characters.

.. code-block:: c++

    // Using Builders
    DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
    DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(created_type);
    data->SetStringValue("Dynamic String");

    // Creating directly the Dynamic Type
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
    DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
    data2->SetStringValue("Dynamic String");


Alias
^^^^^

Alias types have been implemented to rename an existing type, keeping the rest of properties
of the given type.
DynamicTypeBuilderFactory exposes the method `CreateAliasType` to create alias types
taking the base type and the new name that the alias is going to set.
After the creation of the DynamicData, users can access to its information like
they were working with the base type.

.. code-block:: c++

    // Using Builders
    DynamicTypeBuilder_ptr base_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
    DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(base_builder.get());
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(created_type.get(), "alias");
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(builder.get());
    data->SetStringValue("Dynamic Alias String");

    // Creating directly the Dynamic Type
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
    DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pType, "alias");
    DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pAliasType);
    data2->SetStringValue("Dynamic Alias String");

Enum
^^^^

The enum type is managed as complex in Dynamic Types, because it allows to add members
to set the different values that the enum is going to manage.
Internally, it works with an `UINT32` to store what value is selected.

To use enums users must create a Dynamic Type builder calling to `CreateEnumType`
and after that they can call to `AddMember` given the index and the name of the
different values that the enum is going to support.

The DynamicData class has got methods `GetEnumValue` and `SetEnumValue` to work
with `UINT32` or with strings using the names of the members added to the builder.

.. code-block:: c++

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateEnumBuilder();
    builder->AddEmptyMember(0, "DEFAULT");
    builder->AddEmptyMember(1, "FIRST");
    builder->AddEmptyMember(2, "SECOND");
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(pType);

    std::string sValue = "SECOND";
    data->SetEnumValue(sValue);
    uint32_t uValue = 2;
    data->SetEnumValue(uValue);

Bitset
^^^^^^

Bitset types emulate a list of boolean values, but optimized for space allocation
using each bit for a different value.
They work like a boolean type with the only difference that the `GetBoolValue` and
`SetBoolValue` need the index of the bit that users want to read or write.

DynamicTypeBuilderFactory offers the possibility to set the maximum value that the
bitset is going to manage, but it should be less or equal to 64 bits.

.. code-block:: c++

    uint32_t limit = 5;

    // Using Builders
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateBitsetBuilder(limit);
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(pType);
    data->SetBoolValue(true, 2);
    bool bValue;
    data->GetBoolValue(bValue, 0);

    // Creating directly the Dynamic Type
    DynamicType_ptr pType2 = DynamicTypeBuilderFactory::GetInstance()->CreateBitsetType(limit);
    DynamicData_ptr data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
    data2->SetBoolValue(true, 2);
    bool bValue2;
    data2->GetBoolValue(bValue2, 0);

Bitmask
^^^^^^^

Bitmasks are the complex way to work with bitsets because they opens the option to
add members and access to each boolean value with the name of the member.
DynamicData has the special methods `GetBitmaskValue` and `SetBitmaskValue`
using the name of the member, but they can be used like bitsets too.

.. code-block:: c++

    uint32_t limit = 5;

    // Using Builders
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateBitmaskBuilder(limit);
    builder->AddEmptyMember(0, "FIRST");
    builder->AddEmptyMember(1, "SECOND");
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(pType);
    data->SetBoolValue(true, 2);
    bool bValue;
    data->GetBoolValue(bValue, 0);
    bValue = data->GetBitmaskValue("FIRST");

Structure
^^^^^^^^^

Structures are the common complex types, they allow to add any kind of members
inside them.
They don't have any value, they are only used to contain other types.

To manage the types inside the structure, users can call the Get and Set methods
according with the kind of the type inside the structure using their ids.
If the structure contains a complex value, it should be used with `LoanValue` to
access to it and `ReturnLoanedValue` to release that pointer.
DynamicData manages the counter of loaned values and users can't loan a value that
has been loaned previously without calling `ReturnLoanedValue` before.

The Ids must be consecutive starting by zero, and the DynamicType will change that
Id if it doesn't match with the next value.
If two members have the same Id, after adding the second one, the previous
will change its id to the next value.
To get the id of a member by name, DynamicData exposes the method `GetMemberIdByName`.

.. code-block:: c++

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
    builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
    builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

    DynamicType_ptr struct_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(struct_type);

    data->SetInt32Value(5, 0);
    data->SetUint64Value(13, 1);

Union
^^^^^

Unions are a special kind of structures where only one of the members is active
at the same time.
To control these members, users must set the discriminator type that is going to be used
to select the current member calling the `CreateUnionType` method.
After the creation of the Dynamic Type, every member that is going to be added
needs at least one UnionCaseIndex to set how it is going to be selected and
optionally if it is the default value of the union.

.. code-block:: c++

    DynamicType_ptr discriminator = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(discriminator.get());

    builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type(), "", { 0 }, true);
    builder->AddMember(0, "second", DynamicTypeBuilderFactory::GetInstance()->CreateInt64Type(), "", { 1 }, false);
    DynamicType_ptr union_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(union_type);

    data->SetInt32Value(9, 0);
    data->SetInt64Value(13, 1);
    uint64_t unionLabel;
    data->GetUnionLabel(unionLabel);

Sequence
^^^^^^^^

Complex type that manages its members as a list of items allowing users to
insert, remove or access to a member of the list. To create this type users
need to specify the type that it is going to store and optionally the size
limit of the list.
To ease the memory management of this type, DynamicData has these methods:
- `InsertSequenceData`: Creates a new element at the end of the list and returns
the id of the new element.
- `RemoveSequenceData`: Removes the element of the given index and refresh the ids
to keep the consistency of the list.
- `ClearData`: Removes all the elements of the list.

.. code-block:: c++

    uint32_t length = 2;

    DynamicType_ptr base_type = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(base_type.get(), length);
    DynamicType_ptr sequence_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(sequence_type);

    MemberId newId, newId2;
    data->InsertInt32Value(10, newId);
    data->InsertInt32Value(12, newId2);
    data->RemoveSequenceData(newId);

Array
^^^^^

Arrays are pretty similar to sequences with two main differences. The first one is
that they can have multiple dimensions and the other one is that they don't need
that the elements are stored consecutively.
The method to create arrays needs a vector of sizes to set how many dimensions are
going to be managed, if users don't want to set a limit can set the value as zero
on each dimension and it applies the default value ( 100 ).
To ease the management of arrays every `Set` method in DynamicData class creates
the item if there isn't any in the given Id.
Arrays also have methods to handle the creation and deletion of elements like
sequences, they are `InsertArrayData`, `RemoveArrayData` and `ClearData`.
Additionally, there is a special method `GetArrayIndex` that returns the position id
giving a vector of indexes on every dimension that the arrays supports, that is
useful in multidimensional arrays.

.. code-block:: c++

    std::vector<uint32_t> lengths = { 2, 2 };

    DynamicType_ptr base_type = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(base_type.get(), lengths);
    DynamicType_ptr array_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(array_type);

    MemberId pos = data->GetArrayIndex({1, 0});
    data->SetInt32Value(11, pos);
    data->SetInt32Value(27, pos + 1);
    data->ClearArrayData(pos);

Map
^^^

Maps contains a list of pairs 'key-value' types, allowing users to insert, remove or
modify the element types of the map. The main difference with sequences is that the map
works with pairs of elements and creates copies of the key element to block the access
to these elements.

To create a map, users must set the types of the key and the value elements and
optionally the size limit of the map. To add a new element to the map, DynamicData
has the method `InsertMapData` that returns the ids of the key and the value
elements inside the map.
To remove an element of the map there is the method `RemoveMapData` that uses the
given id to find the key element and removes the key and the value elements from the map.
The method `ClearData` removes all the elements from the map.

.. code-block:: c++

    uint32_t length = 2;

    DynamicType_ptr base = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(base.get(), base.get(), length);
    DynamicType_ptr map_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(map_type);

    DynamicData_ptr key = DynamicDataFactory::GetInstance()->CreateData(base);
    MemberId keyId;
    MemberId valueId;
    data->InsertMapData(key.get(), keyId, valueId);
    MemberId keyId2;
    MemberId valueId2;
    key->SetInt32Value(2);
    data->InsertMapData(key.get(), keyId2, valueId2);

    data->SetInt32Value(53, valueId2);

    data->RemoveMapData(keyId);
    data->RemoveMapData(keyId2);

Complex examples
----------------

Structs with Structs
^^^^^^^^^^^^^^^^^^^^

Structures allow to add other structs inside them, but users must take care that
to access to these members they need to call `LoanValue` to get a pointer to the
data and release it calling `ReturnLoanedValue`.
DynamicDatas manages the counter of loaned values and users can't loan a value that
has been loaned previously without calling `ReturnLoanedValue` before.

.. code-block:: c++

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
    builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
    builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());
    DynamicType_ptr struct_type = builder->Build();

    DynamicTypeBuilder_ptr parent_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
    parent_builder->AddMember(0, "child_struct", struct_type);
    parent_builder->AddMember(1, "second", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(parent_builder.get());

    DynamicData* child_data = data->LoanValue(0);
    child_data->SetInt32Value(5, 0);
    child_data->SetUint64Value(13, 1);
    data->ReturnLoanedValue(child_data);

Structs inheritance
^^^^^^^^^^^^^^^^^^^

Structures can inherit from other structures. To do that DynamicTypeBuilderFactory
has the method `CreateChildStructType` that relates the given struct type with
the new one. The resultant type contains the members of the base class and the ones
that users have added to it.

Structures support several levels of inheritance, creating recursivelly the members
of all the types in the hierarchy of the struct.

.. code-block:: c++

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
    builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
    builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

    DynamicTypeBuilder_ptr child_builder = DynamicTypeBuilderFactory::GetInstance()->CreateChildStructBuilder(builder.get());
    builder->AddMember(2, "third", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

    DynamicType_ptr struct_type = child_builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(struct_type);

    data->SetInt32Value(5, 0);
    data->SetUint64Value(13, 1);
    data->SetUint64Value(47, 2);

Alias of an alias
^^^^^^^^^^^^^^^^^

Alias types support recursivity, so if users need to create alias of another alias,
it can be done calling `CreateAliasType` method giving the alias as base type.

.. code-block:: c++

    // Using Builders
    DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
    DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(created_builder.get(), "alias");
    DynamicTypeBuilder_ptr builder2 = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(builder.get(), "alias2");
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(builder2.get());
    data->SetStringValue("Dynamic Alias 2 String");

    // Creating directly the Dynamic Type
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
    DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pType, "alias");
    DynamicType_ptr pAliasType2 = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pAliasType, "alias2");
    DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pAliasType);
    data2->SetStringValue("Dynamic Alias 2 String");

Unions with complex types
^^^^^^^^^^^^^^^^^^^^^^^^^

Unions support complex types, the available interface to access to them is calling
`LoanValue` to get a pointer to the data and set this field as the active one and
release it calling `ReturnLoanedValue`.

.. code-block:: c++

    DynamicType_ptr discriminator = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(discriminator.get());
    builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type(), "", { 0 }, true);

    DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
    struct_builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
    struct_builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());
    builder->AddMember(1, "first", struct_builder.get(), "", { 1 }, false);

    DynamicType_ptr union_type = builder->Build();
    DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(union_type);

    DynamicData* child_data = data->LoanValue(1);
    child_data->SetInt32Value(9, 0);
    child_data->SetInt64Value(13, 1);
    data->ReturnLoanedValue(child_data);

Serialization
-------------

Dynamic Types have their own pubsub type like any class generated with an IDL, and
their management is pretty similar to them.

.. code-block:: c++

    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicPubSubType pubsubType(pType);

    // SERIALIZATION EXAMPLE
    DynamicData* pData = DynamicDataFactory::GetInstance()->CreateData(pType);
    uint32_t payloadSize = static_cast<uint32_t>(pubsubType.getSerializedSizeProvider(data)());
    SerializedPayload_t payload(payloadSize);
    pubsubType.serialize(data, &payload);

    // DESERIALIZATION EXAMPLE
    types::DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
    pubsubType.deserialize(&payload, data2);

Important Notes
---------------

The most important part about Dynamic Types is the memory management, because
every dynamic type and dynamic data are managed with pointers. Every object stored
inside of other dynamic object is managed by its owner, so users only must  take care
of the objects that they have created calling to the factories.
These two factories in charge to manage these objects, and they must create and delete every object.

.. code-block:: c++

    DynamicTypeBuilder* pBuilder = DynamicTypeBuilderFactory::GetInstance()->CreateUint32Builder();
    DynamicType* pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicData* pData = DynamicDataFactory::GetInstance()->CreateData(pType);

    DynamicTypeBuilderFactory::GetInstance()->DeleteType(pType);
    DynamicTypeBuilderFactory::GetInstance()->DeleteBuilder(pBuilder);
    DynamicDataFactory::GetInstance()->DeleteData(pData);

To ease this management, the library incorporates special shared pointers to call
to the factories to delete the object directly ( `DynamicTypeBuilder_ptr`,
`DynamicType_ptr` and  `DynamicData_ptr`).
The only restriction about using this kind of pointers are
the methods `LoanValue` and `ReturnLoanedValue`, because they return a pointer
to an object that is already managed by the library and using a `DynamicData_ptr`
with them will cause a crash.

.. code-block:: c++

    DynamicTypeBuilder_ptr pBuilder = DynamicTypeBuilderFactory::GetInstance()->CreateUint32Builder();
    DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
    DynamicData_ptr pData = DynamicDataFactory::GetInstance()->CreateData(pType);


Dynamic Types Discovery and Endpoint Matching
---------------------------------------------

When using Dynamic Types support, Fast-RTPS make use of an optional TopicDiscoveryKind QoS Policy and TypeIdV1.
At its current state, the matching will only verify that both endpoints are using the same topic type,
but will not negotiate about it.

This verification is done through `MinimalTypeObject`.

TopicDiscoveryKind
^^^^^^^^^^^^^^^^^^

TopicAttribute to indicate which kind of Dynamic discovery we are using.
Can take 3 different values:

**NO_CHECK**: Default value. Will not perform any check for dynamic types.

**MINIMAL**: Will check only at TypeInformation level (and MinimalTypeObject if needed).

**COMPLETE**: Will perform a full check with CompleteTypeObject.

TypeObject (TypeObjectV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

There are two kinds of TypeObject: MinimalTypeObject and CompleteTypeObject.

 - **MinimalTypeObject** is used to check compatibility between types.
 - **CompleteTypeObject** fully describes the type.

Both are defined in the annexes of DDS-XTypes V1.2 document so its details will not be covered in this document.

 - **TypeObject** is an IDL union with both representation, *Minimal* and *Complete*.

TypeIdentifier (TypeIdV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

TypeIdentifier is described too in the annexes of DDS-XTypes V1.2 document.
It represents a full description for basic types and has an EquivalenceKind for complex ones.
An EquivalenceKind is a hash code of 14 octet, as described by the DDS-XTypes V1.2 document.

TypeObjectFactory
^^^^^^^^^^^^^^^^^

Singleton class that manages the creation and access for all registered TypeObjects and TypeIdentifiers.
From a basic TypeIdentifier (in other words, a TypeIdentifier whose discriminator isn't EK_MINIMAL or EK_COMPLETE)
can generate a full DynamicType.

Fastrtpsgen
^^^^^^^^^^^

FastRTPSGen has been upgraded to generate XXXTypeObject.h and .cxx files, taking XXX as our IDL type.
These files provides a small Type Factory for the type XXX.
Generally this files are not used directly, as now the type XXX will register itself through its factory to
TypeObjectFactory in its constructor, making very easy the use of static types with dynamic types.


XML Dynamic Types
-----------------

XML Dynamic Types allows eProsima Fast-RTPS to create Dynamic Types directly defining them through XML.
This allows any application to change TopicDataTypes without the need to change its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (`<types>`, types tag) in the XML file can be placed similary to the profiles tag.
It can be a stand-alone XML Types file or be a child of the fastrtps xml root tag (`<dds>`).
Inside the types tag, must be one or more type tags (`<type>`).

Stand-Alone:

.. code-block:: xml

    <types>
        <type>
            [Type definition]
        </type>
        <type>
            [Type definition]
            [Type definition]
        </type>
    </types>

Rooted:

.. code-block:: xml

    <dds>
        <types>
            <type>
                [Type definition]
            </type>
            <type>
                [Type definition]
                [Type definition]
            </type>
        </types>
    </dds>

Finally each type tag can contain one or more Type definition.
Define several types inside a type tag or each type in its own type tag has the same result.

Type definition
^^^^^^^^^^^^^^^

**Enum**

The enum type is defined by its name and a set of literals, each of them with its name and its (optional) value.

Example:

.. code-block:: xml

    <enum name="MyEnum">
        <literal name="A" value="0"/>
        <literal name="B" value="1"/>
        <literal name="C" value="2"/>
    </enum>

**Typedef**

The typedef type is defined by its name and its value or an inner element for complex types.

Example:

.. code-block:: xml

    <typedef name="MyAlias1" value="MyEnum"/>

    <typedef name="MyAlias2">
        <long dimensions="2,2"/>
    </typedef>

Typedefs correspond to Alias in Dynamic Types glosary.

**Struct**

The struct type is defined by its name and inner members.

Example:

.. code-block:: xml

    <struct name="MyStruct">
        <long name="first"/>
        <longlong name="second"/>
    </struct>

**Union**

The union type is defined by its name, a discriminator and a set of cases.
Each case have one or more caseValue and a member.


Example:

.. code-block:: xml

    <union name="MyUnion">
        <discriminator type="octet"/>
        <case>
            <caseValue value="0"/>
            <caseValue value="1"/>
            <long name="first"/>
        </case>
        <case>
            <caseValue value="2"/>
            <MyStruct name="second"/>
        </case>
        <case>
            <caseValue value="default"/>
            <longlong name="third"/>
        </case>
    </union>

Member types
^^^^^^^^^^^^

By member types we refer to any type that can belong to a struct or an union, or be aliased by a typedef.

When used as sequences elements, key or value types of a map, as an aliased type, etc., its name attribute
is ignored and can be omited.

**Basic types**

The available basic types xml tags are:

- boolean
- octet
- char
- wchar
- short
- long
- longlong
- unsignedshort
- unsignedlong
- unsignedlonglong
- float
- double
- longdouble
- string
- wstring
- boundedString
- boundedWString

All of them are defined simply:

.. code-block:: xml

    <longlong name="my_long"/>

Except boundedString and boundedWString that an inner element *maxLength* whose value indicates
the maximum length of the string.

.. code-block:: xml

    <boundedString name="my_large_string">
        <maxLength value="41925"/>
    </boundedString>


**Arrays**

Arrays are defined exactly the same way as any other member type, but adds the attribute *dimensions*.
The format of this dimensions attribute is the size of each dimension separated by commas.

Example:

.. code-block:: xml

    <long name="long_array" dimensions="2,3,4"/>

It's IDL analog would be:

.. code-block:: c++

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its name, its content type and its (optional) length.
The type of its content can be defined by its type attribute or by a member type.

Example:

.. code-block:: xml

    <sequence name="my_sequence_sequence" length="3">
        <sequence type="long" length="2"/>
    </sequence>

The example shows a sequence with length 3 of sequences with length 2 with long contents.
As IDL would be:

.. code-block:: c++

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner (or content) sequence has no name, due it would be ignored by the parser.

**Maps**

Maps are similar to sequences but they need to define two types instead one. One for its key and another
for its value.
Again, both types can be defined as attributes or as members, but in this cases, when defined
as members they are content in another xml element key_type and value_type respectively.

The definition kind of each type can be mixed, this is, one type can be defined as attribute and the
other as member.

Example:

.. code-block:: xml

    <map name="my_map_map" key_type="long" length="2">
        <value_type>
            <map key_type="long" value_type="long" length="2"/>
        </value_type>
    </map>

Is equivalent to the IDL:

.. code-block:: c++

    map<long,map<long,long,2>,2> my_map_map;

**Complex types**

Once defined, complex types can be used as members in the same way a basic or array type would be.

Example:

.. code-block:: xml

    <struct name="OtherStruct">
        <MyEnum name="my_enum"/>
        <MyStruct name="my_struct" dimensions="5"/>
    </struct>

Usage
^^^^^

In the application that will make use of XML Types, we need to load the xml file that defines our types,
and then, simply instantiate DynamicPubSubTypes of our desired type.

Remeber that only Structs generate usable DynamicPubSubType instances.

.. code-block:: cpp

    // Load the XML File
    XMLP_ret ret = XMLProfileManager::loadXMLFile("types.xml");
    // Create the "MyStructPubSubType"
    DynamicPubSubType *pbType = XMLProfileManager::CreateDynamicPubSubType("MyStruct");
    // Create a "MyStruct" instance
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(pbType->GetDynamicType());
