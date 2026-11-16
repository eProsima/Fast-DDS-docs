.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fast_dds_suite:

Fast DDS Suite Image
====================

eProsima provides the *Fast DDS Suite* Docker image for those who want a Docker image with a set of eProsima's tools
and libraries running on an Ubuntu platform.
It can be downloaded from `eProsima's downloads page <https://eprosima.com/index.php/downloads-all>`_.

This Docker image contains the complete Fast DDS suite. This includes:

- :ref:`eProsima Fast DDS library and examples <eprosima_dds_suite_examples>`: *Fast DDS* library bundled with several
  examples that showcase a variety of capabilities of eProsima's *Fast DDS* implementation.
  For more information about *Fast DDS* please refer to `Fast DDS documentation page
  <https://fast-dds.docs.eprosima.com/>`_.

- :ref:`DDS Monitor <eprosima_dds_suite_monitor>`: *eProsima DDS Monitor* is a graphical desktop application
  aimed at monitoring DDS environments deployed using the *eProsima Fast DDS* library, which contains an open-source
  version of the *DDS Monitor*.
  Thus, the user can monitor the real time status of publication/subscription communications between DDS entities.
  They can also choose which communication parameters are to be measured (latency, throughput,
  packet loss, etc.), as well as record and compute real time statistical measurements on these parameters
  (mean, variance, standard deviation, etc.).
  For more information about *DDS Monitor* please refer to `DDS Monitor documentation page
  <https://dds-monitor.docs.eprosima.com/>`_.

- :ref:`DDS Router <eprosima_dds_router>`: *eProsima DDS Router* is an end-user software application that enables the
  connection of distributed DDS networks.
  That is, DDS entities such as publishers and subscribers deployed in one geographic location and using a dedicated
  local network will be able to communicate with other DDS entities deployed in different geographic areas on their own
  dedicated local networks as if they were all on the same network through the use of eProsima DDS Router.
  For more information about *DDS Router* please refer to
  `DDS Router documentation website <https://eprosima-dds-router.readthedocs.io>`_.

- :ref:`DDS Record & Replay <eprosima_dds_record_replay>`:
  *eProsima DDS Record & Replay* is an end-user software application that efficiently saves DDS data published into
  a DDS environment in a MCAP or SQLite format database.
  Thus, the exact playback of the recorded network events is possible as the data is linked to the timestamp at which
  the original data was published.
  For more information about *eProsima DDS Record & Replay* please refer to
  `DDS Record & Replay documentation website <https://dds-recorder.readthedocs.io>`_.

- :ref:`Fast DDS Spy <eprosima_fast_dds_spy>`:
  *eProsima Fast DDS Spy* is a CLI interactive tool that allows to introspect a DDS network in human readable format.
  It is possible to query the network about the DomainParticipants connected, their endpoints
  (DataWriters and DataReaders) and the topics they communicate in, as well as see the data being sent through the
  network.
  For more information about *eProsima Fast DDS Spy* please refer to
  `Fast DDS Spy documentation website <https://fast-dds-spy.readthedocs.io>`_.

- :ref:`Shapes Demo <eprosima_dds_suite_shapes_demo>`: *eProsima Shapes Demo* is an application in which Publishers and
  Subscribers create and display shapes of different colors and sizes moving on a board.
  Each shape refers to its own topic: Square, Triangle or Circle.
  A single instance of the *eProsima Shapes Demo* can publish on and/or subscribe to several topics at a time.
  For more information about *Shapes Demo* please refer to `Shapes Demo documentation page
  <https://eprosima-shapes-demo.readthedocs.io/>`_.

To load this image into your Docker repository, from a terminal run

.. code-block:: bash

    docker load -i "ubuntu-fastdds-suite_<fastdds-version>.tar"

You can run this Docker container as follows

.. code-block:: bash

    xhost local:root
    docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    ubuntu-fastdds-suite:<FastDDS-Version>

From the resulting Bash Shell you can run each feature.

.. _fast_dds_suite_examples:

.. _eprosima_dds_suite_examples:

Fast DDS Examples
-----------------

Included in this Docker container is a set of binary examples that showcase several functionalities of the
*Fast DDS* library.
These examples' path can be accessed from a terminal by typing

.. code-block:: bash

    goToExamples

This will change the working directory to a location containing several examples, both for DDS and RTPS.
Below are the steps to launch two such examples.

Hello World Example
^^^^^^^^^^^^^^^^^^^

This is a minimal example that will perform a Publisher/Subscriber match and start sending samples.

.. code-block:: bash

    goToExamples
    cd hello_world/bin
    tmux new-session \
        "./hello_world publisher 0 1000" \; \
        split-window "./hello_world subscriber" \; \
        select-layout even-vertical

This example is not constrained to the current image instance, meaning that it is possible to run several instances of
this container to check the communication between them.
From one terminal you could launch an image and, on the presented shell, run:

.. code-block:: bash

    goToExamples
    cd hello_world/bin
    ./hello_world publisher

And then from another terminal with another instance run the following:

.. code-block:: bash

    goToExamples
    cd hello_world/bin
    ./hello_world subscriber

Benchmark Example
^^^^^^^^^^^^^^^^^

This example creates either a Publisher or a Subscriber and, after a successful match, starts sending samples.
After a few seconds the process that launched the Publisher will show a report with the number of samples transmitted.

On the subscriber side, run:

.. code-block:: bash

    goToExamples
    cd benchmark/bin
    ./benchmark subscriber udp

On the publisher side, run:

.. code-block:: bash

    goToExamples
    cd benchmark/bin
    ./benchmark publisher udp

.. _eprosima_dds_suite_monitor:

DDS Monitor
-----------

To launch *DDS Monitor*, from a terminal run:

.. code-block:: bash

    fastdds_monitor

*eProsima DDS Monitor* User Manual can be located on the `DDS Monitor documentation
<https://dds-monitor.docs.eprosima.com/en/latest/rst/user_manual/initialize_monitoring.html>`_.

.. _eprosima_dds_router:

DDS Router
----------

In this example the DDS Router is configured to communicate a publisher and subscriber running in different DDS Domains.

Run the following command to create the DDS Router *yaml* configuration file (``/config.yml``).

.. code-block:: bash

    echo "version: v2.0
    participants:
      - name: simple_dds_participant_0
        kind: local
        domain: 0
      - name: simple_dds_participant_1
        kind: local
        domain: 1" > /config.yml

Then execute the following command to run the Publisher in Domain 0, the Subscriber in Domain 1, and the
DDS Router communicating both Domains.

.. code-block:: bash

    goToExamples
    cd configuration/bin
    tmux new-session \
        "ddsrouter --config-path /config.yml" \; \
        split-window -h "./configuration publisher --domain 0 --interval 1000 --transport udp" \; \
        split-window -v "./configuration subscriber --domain 1 --transport udp"

.. _eprosima_dds_record_replay:

DDS Record & Replay
-------------------

DDS Record & Replay is composed of two different tools: DDS Recorder and DDS Replayer.
DDS Recorder allows to record DDS traffic in a specific domain, storing all data in an MCAP file.
To launch DDS Recorder, from a terminal run:

.. code-block:: bash

 $ ddsrecorder

Recorded data can then be inspected through visualization applications such as `Foxglove Studio
<https://foxglove.dev/>`_.
It is also possible to play data back in the same domain or a different one by leveraging DDS Replayer.
To launch DDS Replayer, from a terminal run:

.. code-block:: bash

 $ ddsreplayer -i my_data.mcap

For more information on how to configure and use DDS Record & Replay, please refer to
`DDS Record & Replay documentation website <https://dds-recorder.readthedocs.io>`_.

.. _eprosima_fast_dds_spy:

Fast DDS Spy
------------

To launch Fast DDS Spy, from a terminal run:

.. code-block:: bash

 $ fastddsspy

Fast DDS Spy usage information can be found on the `Fast DDS Spy User Manual
<https://fast-dds-spy.readthedocs.io/en/latest/rst/user_manual/usage_example.html/>`_.

.. _eprosima_dds_suite_shapes_demo:

Shapes Demo
-----------

To launch the *Shapes Demo*, from a terminal run:

.. code-block:: bash

    ShapesDemo

*eProsima Shapes Demo* usage information can be found on the `Shapes Demo documentation
<https://eprosima-shapes-demo.readthedocs.io/en/latest/first_steps/first_steps.html>`_.
