.. include:: includes/aliases.rst

.. _dds_layer_log_intro:

Logging
=======

Fast DDS provides an extensible built-in logging module that exposes the following functionalities:

* Three different logging levels: |Log::Kind::Info|, |Log::Kind::Warning|, and |Log::Kind::Error|.
* Message filtering according to different criteria: category, content, or file of origin.
* Output to STDOUT and/or log file.

This section is devoted to explain the use, configuration, and extensibility of Fast DDS' :ref:`dds_layer_log_intro`.

.. figure:: /01-figures/fast_dds/log/class_diagram.svg
    :align: center

    Logging module class diagram

.. toctree::

    /fastdds/logging/log
    /fastdds/logging/consumer
