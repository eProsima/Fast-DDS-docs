Code generation using fastrtpsgen
=================================

*eprosima Fast RTPS* comes with a built-in code generation tool, fastrtpsgen, which eases the process of
translating an IDL specification of a data type to a working implementation of the methods needed to create
topics, used by publishers and subscribers, of that data type. This tool can be instructed to generate a sample application using
this data type, providing a Makefile to compile it on Linux and a Visual Studio project for Windows.

*fastrtpsgen* can be invoked by calling fastrtpsgen on Linux or fastrtpsgen.bat on Windows. ::

	fastrtpsgen -d <outputdir> -example <platform> -replace <IDLfile>

The `-replace` argument is needed to replace the currently existing files in case the files for the IDL have been
generated previously.

When the `-example` argument is added, the tool will generate an automated example and the files to build
it for the platform currently invoked. The `-help` argument provides a list of currently supported Visual Studio
versions and platforms.

Output
------

*fastrtpsgen* outputs the several files. Assuming the IDL file had the name *“Mytype”*, these files are:

* MyType.cxx/.h: Type definition.
* MyTypePublisher.cxx/.h: Definition of the Publisher as well as of a PublisherListener. The user must fill the needed methods for his application.
* MyTypeSubscriber.cxx/.h: Definition of the Subscriber as well as of a SubscriberListener. The behavior of the subscriber can be altered changing the methods implemented on these files.
* MyTypePubSubType.cxx/.h: Serialization and Deserialization code for the type. It also defines the getKey method in case the topic uses keys.
* MyTypePubSubMain.cxx: Main file of the example application in case it is generated.
* Makefiles or Visual studio project files.

Where to find *fastrtpsgen*
---------------------------

If you are using the binary distribution of *eProsima Fast RTPS*, *fastrtpsgen* is already provided for you.
If you are building from sources, you have to compile *fastrtpsgen*. You can find instructions in section :ref:`installation-from-sources`.

