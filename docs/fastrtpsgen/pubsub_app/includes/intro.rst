First of all you need to follow the steps outlined in the :ref:`installation` for the installation of
*eprosima Fast RTPS* and all its dependencies.
If you have followed the steps, you should have Fast DDS, Fast CDR, and Fast-RTPS-Gen installed.

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

Now we need to translate this file to something Fast DDS understands.
For this we have a code generation tool called Fast DDS-Gen (see :ref:`fastrtpsgen_use`), which can do two different
things:

1. Generate C++ definitions for your custom topic.
2. Generate a functional example that uses your topic data.

It is the second option which is used to create this publish/subscribe application, while the first option is applied
in this other tutorial: :ref:`writing_pubsub_app`.

