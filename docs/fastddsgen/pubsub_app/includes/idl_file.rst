Creating the IDL file with the data type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build a minimal application, you must first define the Topic.
To do this, the data type of the Topic is defined by means of an IDL file.
An example of this is shown below. In this case, a single string is sufficient.
Topics are explained in more detail in :ref:`dds_layer_topic`, while the topic data types to be defined using IDL are
presented in :ref:`dds_layer_definition_data_types`.

.. code:: bash

    mkdir HelloWorldExample && cd HelloWorldExample


.. code-block:: idl

    // HelloWorld.idl
    struct HelloWorld
    {
        string message;
    };

Then, this file is translated to something *Fast DDS* understands.
For this, use the Fast DDS-Gen code generation tool, which can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate a functional example that uses your topic data.

It is the second option which is used to create this publish/subscribe application, while the first option is applied
in this other tutorial: :ref:`writing_pubsub_app`.

