<a id="fastddsgen-intro"></a>

<a id="introduction"></a>

# Introduction

*eProsima Fast DDS-Gen* is a Java application that generates *eProsima Fast DDS* source code using the data types
defined in an IDL (Interface Definition Language) file.
This generated source code can be used in any *Fast DDS* application in order to define the data type of a topic,
which will later be used to publish or subscribe.
*eProsima Fast DDS* defines the data type exchanged in a Topic through two classes: the [`TypeSupport`](../../fastdds/api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) and the
[`TopicDataType`](../../fastdds/api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE). [`TopicDataType`](../../fastdds/api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) describes the data type exchanged between a publication and a subscription, i.e.
the data corresponding to a Topic; while [`TypeSupport`](../../fastdds/api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) encapsulates an instance of TopicDataType, providing
the functions needed to register the type and interact with the publication and subscription.
Please refer to [Definition of data types](../../fastdds/dds_layer/topic/typeSupport/typeSupport.md#dds-layer-definition-data-types) for more information on data types.

To declare the structured data, the IDL format must be used.
IDL is a specification language, made by [OMG](https://www.omg.org/) (Object Management Group), which describes an
interface in a language independent manner, allowing communication between software components that do not share the
same language.
The *eProsima Fast DDS-Gen* tool reads the IDL files and parses a subset of the
[OMG IDL specification](https://www.omg.org/spec/IDL/4.2/) to generate
source code for data serialization.
This subset includes the data type descriptions included in [Defining a data type via IDL](../dataTypes/dataTypes.md#fastddsgen-idl-datatypes).
The rest of the file content is ignored.

*eProsima Fast DDS-Gen* generated source code uses [Fast CDR](https://github.com/eProsima/Fast-CDR), a C++11 library
that provides the data serialization and codification mechanisms.
Therefore, as stated in the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF), when the data are sent,
they are serialized and encoded using the corresponding Common Data Representation (CDR).
The CDR transfer syntax is a low-level representation for inter-agents transfer, mapping from OMG IDL data types to
byte streams.
Please refer to the official [CDR specification](https://www.omg.org/cgi-bin/doc?formal/02-06-51) for more
information on the CDR transfer syntax (see PDF section 15.3).

The main feature of *eProsima Fast DDS-Gen* is to facilitate the implementation of DDS applications without the
knowledge of serialization or deserialization mechanisms.
With *Fast DDS-Gen* it is also possible to generate the C++ source code of a DDS application with a publisher and a
subscriber that uses the *eProsima Fast DDS* library (see [Building a publish/subscribe application](../pubsub_app/pubsub_app.md#fastddsgen-pubsub-app)).
*Fast DDS-Gen* can also generate Python bindings for the data types in order to use them within a Python-based
*Fast DDS* application (see [Building Python auxiliary libraries](../python_bindings/python_bindings.md#fastddsgen-python-bindings)).

For installing *Fast DDS-Gen*, please refer to [Linux installation of Fast DDS-Gen](../../installation/sources/sources_linux.md#fastddsgen-sl) or to
[Windows installation of Fast DDS-Gen](../../installation/sources/sources_windows.md#fastddsgen-sw).
