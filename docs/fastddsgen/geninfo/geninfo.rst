.. _fastrtpsgen_intro:

Introduction
============

eProsima Fast DDS-Gen is a Java application that generates *eProsima Fast DDS* source code using the data types defined
in an IDL (Interface Definition Language) file.
This generated source code can be used in any *Fast DDS* application in order to define the data type of a topic,
which will later be used to publish or subscribe.

To declare the structured data, the IDL format must be used.
IDL is a specification language, made by `OMG <https://www.omg.org/>`_ (Object Management Group), which describes an
interface in a language independent manner, allowing communication between software components that do not share the
same language.

The eProsima Fast DDS-Gen tool reads the IDL files and parses a subset of the
`OMG IDL specification <https://www.omg.org/spec/IDL/4.2/>`_ to generate
source code for data serialization.
This subset includes the data type descriptions included in :ref:`idl-types`.
The rest of the file content is ignored.

eProsima Fast DDS-Gen generated source code uses `Fast CDR <https://github.com/eProsima/Fast-CDR>`_, a C++11 library
that provides the data serialization and codification mechanisms.
Therefore, as stated in the RTPs standard, when the data are sent, they are serialized and encoded using the
corresponding Common Data Representation (CDR).
The CDR transfer syntax is a low-level representation for inter-agents transfer, mapping from OMG IDL data types to
byte streams.
Please refer to the official `CDR specification <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ for more
information on the CDR transfer syntax (see PDF section 15.3).

The main feature of eProsima Fast DDS-Gen is to facilitate the implementation of DDS applications without the knowledge
of serialization or deserialization mechanisms.
With Fast DDS-Gen it is also possible to generate the source code of a DDS application with a publisher and a
subscriber that uses the *eProsima Fast DDS* library.

For installing Fast DDS-Gen, please refer to :ref:`Linux installation of Fast DDS-Gen <fastddsgen_sl>` or to
:ref:`Window installation of Fast DDS-Gen <fastddsgen_sw>`.
