First of all you need to follow the steps outlined in :ref:`installation_requirements`
for the installation of required packages, and in :ref:`installation_binaries` or
:ref:`installation-from-sources` for the installation of
*eprosima Fast RTPS* and all its dependencies.
If you have followed the steps, you should have Fast-RTPS, Fast-CDR, and Fast-RTPS-Gen installed.

To build a minimal application, you must first define the Topic.
To do this, the data type of the Topic is defined by means of an IDL file.
An example of this is shown below. In this case, a single string is sufficient.
Topics are explained in more detail in :ref:`dds_layer_topic`, while the topic data types to be defined using IDL are
presented in :ref:`fastrtpsgen_intro`.

.. code:: bash

    mkdir HelloWorldExample && cd HelloWorldExample


.. code-block:: idl

    // HelloWorld.idl
    struct HelloWorld
    {
        string message;
    };

Now we need to translate this file to something Fast RTPS understands.
For this we have a code generation tool called fastrtpsgen (see :ref:`fastrtpsgen_intro`), which can do two different
things:

1. Generate a functional example that uses your topic data.
2. Generate C++ definitions for your custom topic.

The first option is used to create this publish/subscribe application, while the second option is applied in this
other tutorial: :ref:`writing_pubsub_app`.

