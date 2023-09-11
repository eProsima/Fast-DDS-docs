Creating the IDL file with the data type
----------------------------------------

To build a minimal application, the Topic must be defined by means of an IDL file.
For this example the Topic data type defined by IDL is just a ``string`` message.
Topics are explained in more detail in :ref:`dds_layer_topic`, while the Topic data types to be defined using IDL are
presented in :ref:`dds_layer_definition_data_types`.
In the preferred text editor, create the *HelloWorld.idl* file with the following content and save it in the
*FastDDSGenHelloWorld* directory.

.. code-block:: omg-idl

    // HelloWorld.idl
    struct HelloWorld
    {
        string message;
    };

Then, this file is translated to something *Fast DDS* understands.
For this, use the *Fast DDS-Gen* code generation tool, which can do two different things:

1. Generate C++ definitions for a custom topic.
2. Generate a functional example that uses the topic data.

The second option is the one used to create this publish/subscribe application, while the first option is applied
in this other tutorial: :ref:`writing_pubsub_app`.
