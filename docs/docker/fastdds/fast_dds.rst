.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fast_dds:

Fast DDS Image
==============

This Docker image contains the Fast DDS library and its dependencies, ready to be used in a final user application.
This includes:

- :ref:`eProsima Fast DDS libraries and examples <fast_dds_suite_examples>`: Fast DDS libraries bundled with several
  examples that showcase a variety of capabilities of eProsima's Fast DDS implementation.

To load this image into your Docker repository, from a terminal, run:

.. code-block:: bash

    docker load -i "ubuntu-fastdds <FastDDS-Version>.tar"

You can run this Docker container as follows:

.. code-block:: bash

    docker run -it ubuntu-fastdds:<FastDDS-Version>

From the resulting Bash Shell you can run each feature.

.. _fast_dds_examples:

Fast DDS Examples
-----------------

Included in this Docker container is a set of binary examples that showcase several functionalities of the
Fast DDS libraries.
These examples' path can be accessed from a terminal by typing:

.. code-block:: bash

    goToExamples

From this folder, you can access all examples, both for DDS and RTPS layers.

Hello World Example
^^^^^^^^^^^^^^^^^^^

This is a minimal example that will perform a Publisher/Subscriber match and start sending samples.

.. code-block:: bash

   goToExamples
   cd dds/HelloWorldExample/bin
   tmux new-session "./DDSHelloWorldExample publisher 0 1000" \; \
        split-window "./DDSHelloWorldExample subscriber" \; \
        select-layout even-vertical

This example is not constrained to the current instance.
It's possible to run several instances of this container to check the communication between them by running the
following from each container.

.. code-block:: bash

    goToExamples
    cd dds/HelloWorldExample/bin
    ./DDSHelloWorldExample publisher

or

.. code-block:: bash

    goToExamples
    cd dds/HelloWorldExample/bin
    ./DDSHelloWorldExample subscriber
