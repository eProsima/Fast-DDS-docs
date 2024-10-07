.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _dds_record_and_replay:

How to use eProsima DDS Record and Replay
=========================================

`eProsima DDS Record and Replay <https://github.com/eProsima/DDS-Record-Replay>`_ is an end-user software application
that efficiently saves DDS data published into a DDS environment in a MCAP format database.
Thus, the exact playback of the recorded network events is possible as the data is linked to the
timestamp at which the original data was published. This facilitates the debugging of DDS networks.

Getting started
---------------

eProsima DDS Record & Replay includes the following tools:

- **DDS Recorder tool**:  This tool's primary function is to store data in an MCAP database,
  including the publication timestamp, serialized data, and its format.
  The output MCAP file can be read by any compatible tool,
  as it contains all necessary information for data reproduction.

- **DDS Remote Controller tool**: This application enables remote control of the recording tool,
  allowing a user to start, stop, or pause data recording from another device.

- **DDS Replay tool**: This application allows replay of DDS traffic recorded with a DDS Recorder.
  Users can select messages to replay by setting a time range or by blocking/whitelisting specific topics.
  They can also adjust the playback rate and use different topic QoS settings from the original recording.

Prerequisites
^^^^^^^^^^^^^

`eProsima DDS Record and Replay <https://github.com/eProsima/DDS-Record-Replay>`_ depends on
eProsima Fast DDS library and certain Debian packages.
For further information, please refer to the
`installation guide <https://dds-recorder.readthedocs.io/en/latest/rst/installation/docker.html#docker>`_.

Furthermore, the example provided in this section requires
`ShapesDemo <https://eprosima-shapes-demo.readthedocs.io/en/latest/index.html>`_
to publish and subscribe shapes of different colors and sizes.

Example of usage: Recording Application
---------------------------------------

This example will serve as a hands-on tutorial, aimed at introducing some of the key concepts and
features that eProsima DDS Record & Replay recording application offers.

Start ShapesDemo
^^^^^^^^^^^^^^^^

Let us launch a ShapesDemo instance and start publishing in topics ``Square`` with default settings.

.. figure:: /01-figures/fast_dds/recorder_shapesdemo_publisher.png
    :align: center
    :scale: 75 %

Recorder configuration
^^^^^^^^^^^^^^^^^^^^^^

``ddsrecorder`` runs with default configuration settings.
This default configuration records all messages of all DDS Topics found in
DDS Domain ``0`` in the ``output_YYYY-MM-DD-DD_hh-mm-ss.mcap`` file.

Additionally, it is possible to change the default configuration parameters by means of a YAML configuration file.

.. note::
    Please refer to `Configuration <https://dds-recorder.readthedocs.io/en/latest/rst/recording/usage/configuration.html#recorder-usage-configuration>`_
    for more information on how to configure a ``ddsrecorder``.

Recorder execution
^^^^^^^^^^^^^^^^^^

To start recording, execute the following command:

.. code-block:: bash

    ddsrecorder

In order to know all the possible arguments supported by this tool, use the command:

.. code-block:: bash

    ddsrecorder --help


Let us launch a ShapesDemo instance and start publishing in topics ``Square`` with default settings.

.. figure:: /01-figures/fast_dds/recorder_shapesdemo_exec.png

Stop the recorder with ``Ctrl+C`` and check that the MCAP file exists.

Next Steps
==========

The usage of the DDS Remote Controller and DDS Replay tools follows
the same steps as the DDS Recorder tool.

For further information, please refer to the
`eProsima DDS Record and Replay documentation
<https://dds-recorder.readthedocs.io/en/latest/index.html>`_.
