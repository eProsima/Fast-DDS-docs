<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="fastddsgen-interfaces-introduction"></a>

<a id="idl-interfaces-pro"></a>

# IDL interfaces **[Pro]**

*eProsima FastDDS-Gen* tool allows the generation of source code for a [RPC over DDS](../../fastdds/rpc_dds/rpc_dds_intro.md#rpc-dds-intro) application
from an IDL file. The IDL file must contain the operation that can be called on both the client and server sides,
and the parameters that can be passed to them. These operations are specified using the concept of interfaces
defined in the OMG IDL [specification](https://www.omg.org/spec/IDL/4.2/PDF).

The next subsections serves as a guide to how to define your own IDL interface, how to define exceptions for its
operations, and how to develop RPC Client/Server applications.

* [Defining an IDL interface](interfaces.md)
  * [IDL specification overview](interfaces.md#idl-specification-overview)
  * [Defining data streaming operations](interfaces.md#defining-data-streaming-operations)
  * [Example](interfaces.md#example)
* [Exceptions](exceptions.md)
* [Building a RPC Client/Server application](../rpc_calculator_basic_app/intro.md)
  * [Background](../rpc_calculator_basic_app/intro.md#background)
  * [Prerequisites](../rpc_calculator_basic_app/intro.md#prerequisites)
  * [Create the application workspace](../rpc_calculator_basic_app/intro.md#create-the-application-workspace)
  * [Import linked libraries and its dependencies](../rpc_calculator_basic_app/intro.md#import-linked-libraries-and-its-dependencies)
  * [Configure the CMake project](../rpc_calculator_basic_app/intro.md#configure-the-cmake-project)
  * [Build the client/server interface](../rpc_calculator_basic_app/intro.md#build-the-client-server-interface)
  * [Generate the Fast DDS source code from the IDL file](../rpc_calculator_basic_app/intro.md#generate-the-fast-dds-source-code-from-the-idl-file)
  * [Writing the application source code](../rpc_calculator_basic_app/intro.md#writing-the-application-source-code)
  * [Build the application](../rpc_calculator_basic_app/intro.md#build-the-application)
  * [Run the application](../rpc_calculator_basic_app/intro.md#run-the-application)
  * [Next steps](../rpc_calculator_basic_app/intro.md#next-steps)
* [Building a RPC Client/Server application with data streaming](../rpc_calculator_feed_app/intro.md)
  * [Background](../rpc_calculator_feed_app/intro.md#background)
  * [Create the application workspace](../rpc_calculator_feed_app/intro.md#create-the-application-workspace)
  * [Configure the CMake project](../rpc_calculator_feed_app/intro.md#configure-the-cmake-project)
  * [Build the client/server interface](../rpc_calculator_feed_app/intro.md#build-the-client-server-interface)
  * [Generate the Fast DDS source code from the IDL file](../rpc_calculator_feed_app/intro.md#generate-the-fast-dds-source-code-from-the-idl-file)
  * [Writing the application source code](../rpc_calculator_feed_app/intro.md#writing-the-application-source-code)
  * [Build the application](../rpc_calculator_feed_app/intro.md#build-the-application)
  * [Run the application](../rpc_calculator_feed_app/intro.md#run-the-application)
* [Customizing RPC Server request scheduling](../rpc_server_schedule/intro.md)
  * [Example](../rpc_server_schedule/intro.md#example)
