.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fast_dds_examples_docker:

Fast DDS Examples
-----------------

Included in this Docker container is a set of binary examples that showcase several functionalities of the Fast DDS libraries.
These examples path can be accessed from a terminal by typing

 .. code-block:: bash

   $ goToExamples

From this folder you can access all examples, both for DDS and RTPS.

   * **Hello World Example**

   This is a minimal example that will perform a Publisher/Subscriber match and start sending samples.

   .. code-block:: bash

    $ goToExamples
    $ cd HelloWorldExample/bin
    $ tmux new-session "./HelloWorldExample publisher 0 1000" \; \
    split-window "./HelloWorldExample subscriber" \; \
    select-layout even-vertical
  
   This example is not constrained to the current instance. It's possible to run several instances of this container
   to check the communication between them by running the following from each container.

   .. code-block:: bash
 
    $ goToExamples
    $ cd HelloWorldExample/bin
    $ ./HelloWorldExample publisher
  
   or

   .. code-block:: bash

    $ goToExamples
    $ cd HelloWorldExample/bin 
    $ ./HelloWorldExample subscriber

   * **Benchmark Example**
  
   This example creates either a Publisher or a Subscriber and on a successful match starts sending samples. After a few seconds
   the process that launch the Publisher will show a report with the number of samples transmitted.
  
   On the subscriber side, run:

   ..  code-block:: bash

    $ goToExamples
    $ cd Benchmark/bin
    $ ./Benchmark subscriber udp

   On the publisher side, run:

   ..  code-block:: bash

    $ goToExamples
    $ cd Benchmark/bin
    $ ./Benchmark publisher udp
