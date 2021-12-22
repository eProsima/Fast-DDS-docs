.. include:: ../includes/aliases.rst

.. _fastddsgen_intro:

Introduction
============

*eProsima Fast DDS-Gen* is a Java application that generates *eProsima Fast DDS* source code using the data types
defined in an IDL (Interface Definition Language) file.
This generated source code can be used in any *Fast DDS* application in order to define the data type of a topic,
which will later be used to publish or subscribe.
*eProsima Fast DDS* defines the data type exchanged in a Topic through two classes: the |TypeSupport| and the
|TopicDataType|. |TopicDataType| describes the data type exchanged between a publication and a subscription, i.e.
the data corresponding to a Topic; while |TypeSupport| encapsulates an instance of TopicDataType, providing
the functions needed to register the type and interact with the publication and subscription.
Please refer to :ref:`dds_layer_definition_data_types` for more information on data types.

To declare the structured data, the IDL format must be used.
IDL is a specification language, made by `OMG <https://www.omg.org/>`_ (Object Management Group), which describes an
interface in a language independent manner, allowing communication between software components that do not share the
same language.
The *eProsima Fast DDS-Gen* tool reads the IDL files and parses a subset of the
`OMG IDL specification <https://www.omg.org/spec/IDL/4.2/>`_ to generate
source code for data serialization.
This subset includes the data type descriptions included in :ref:`fastddsgen_idl_datatypes`.
The rest of the file content is ignored.

*eProsima Fast DDS-Gen* generated source code uses `Fast CDR <https://github.com/eProsima/Fast-CDR>`_, a C++11 library
that provides the data serialization and codification mechanisms.
Therefore, as stated in the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_, when the data are sent,
they are serialized and encoded using the corresponding Common Data Representation (CDR).
The CDR transfer syntax is a low-level representation for inter-agents transfer, mapping from OMG IDL data types to
byte streams.
Please refer to the official `CDR specification <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ for more
information on the CDR transfer syntax (see PDF section 15.3).

The main feature of *eProsima Fast DDS-Gen* is to facilitate the implementation of DDS applications without the
knowledge of serialization or deserialization mechanisms.
With *Fast DDS-Gen* it is also possible to generate the source code of a DDS application with a publisher and a
subscriber that uses the *eProsima Fast DDS* library (see :ref:`fastddsgen_pubsub_app`).
*Fast DDS-Gen* can also generate Python bindings for the data types in order to use them within a Python-based
*Fast DDS* application (see :ref:`fastddsgen_python_bindings`).

For installing *Fast DDS-Gen*, please refer to :ref:`Linux installation of Fast DDS-Gen <fastddsgen_sl>` or to
:ref:`Windows installation of Fast DDS-Gen <fastddsgen_sw>`.
