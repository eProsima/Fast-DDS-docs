<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-fastddsgen"></a>

<a id="fast-dds-gen-for-data-types-source-code-generation"></a>

# Fast DDS-Gen for data types source code generation

*eProsima Fast DDS* comes with a built-in source code generation tool, *Fast DDS-Gen*, which eases the process of
translating an IDL specification of a data type to a functional implementation.
Thus, this tool automatically generates the source code of a data type defined using IDL.
A basic use of the tool is described below.
To learn about all the features that *Fast DDS* offers, please refer to [Fast DDS-Gen](../../../../fastddsgen/introduction/introduction.md#fastddsgen-intro) section.

<a id="basic-usage"></a>

## Basic usage

*Fast DDS* can be executed by calling *fastddsgen* on Linux or *fastddsgen.bat* on Windows.
The IDL file containing the data type definition is given with the `<IDLfile>` argument.

| **Linux**                                                                |
|--------------------------------------------------------------------------|
| ```bash<br/>fastddsgen [<options>] <IDLfile> [<IDLfile> ...]<br/>```     |
| **Windows**                                                              |
| ```bash<br/>fastddsgen.bat [<options>] <IDLfile> [<IDLfile> ...]<br/>``` |

Among the available arguments defined in [Usage](../../../../fastddsgen/usage/usage.md#fastddsgen-usage), the main *Fast DDS-Gen* options for data type source
code generation are the following:

* `-replace`: It replaces existing files in case the data type files have been previously generated.
* `-help`: It lists the currently supported platforms and Visual Studio versions.
* `-no-typeobjectsupport`: It disables the automatic generation of the [TypeObject representation](../../../xtypes/discovery_matching.md#xtypes-discovery-matching) registration code.
* `-example`: It generates a basic example of a DDS application and the files to build it for
  the given `platform`.
  Thus, *Fast DDS-Gen* tool can generate a sample application using the provided data type, together with a
  Makefile, to compile it on Linux distributions, and a Visual Studio project for Windows.
  To see an example of this please refer to tutorial [Building a publish/subscribe application](../../../../fastddsgen/pubsub_app/pubsub_app.md#fastddsgen-pubsub-app).

<a id="output-files"></a>

## Output files

*Fast DDS-Gen* outputs several files.
Assuming the IDL file had the name  *“Mytype”*, and none of the above options have been defined, these files are:

* MyType.hpp: Type definition.
* MyTypePubSubType.cxx/.hpp: Serialization and deserialization source code for the data type.
  It also defines the `getKey()` member function of the `MyTypePubSubType` class in case the
  topic implements keys (see [Data types with a key](../typeSupport/typeSupport.md#dds-layer-topic-keyed-data-types)).
* MyTypeCdrAux.hpp/.ipp: Auxiliary methods required by Fast CDR for type encoding and decoding.
* MyTypeTypeObjectSupport.cxx/.hpp: Auxiliary code required for [TypeObject representation](../../../xtypes/discovery_matching.md#xtypes-discovery-matching) generation and registration.
