.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_logging_spec:

Log Entry Specification
^^^^^^^^^^^^^^^^^^^^^^^

Log entries created by :ref:`dds_layer_ostream_consumer_stdout`, :ref:`dds_layer_ostream_consumer_stdouterr` and
:ref:`dds_layer_ostream_consumer_file` (*eProsima Fast DDS* built-in :ref:`dds_layer_log_consumer`) adhere to the
following structure:

.. code-block:: bash

    <Timestamp> [<Category> <Verbosity Level>] <Message> (<File Name>:<Line Number>) -> Function <Function Name>

An example of such log entry is given by:

.. code-block:: bash

    2020-05-27 11:45:47.447 [DOCUMENTATION_CATEGORY Error] This is an error message (example.cpp:50) -> Function main

.. note::

    `File Name` and `Line Number`, as well as `Function Name` are only present when enabled. See
    :ref:`dds_layer_log_config` for details.
