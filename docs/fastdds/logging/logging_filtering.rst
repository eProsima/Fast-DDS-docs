.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dds_layer_log_filter:

Filters
-------

*eProsima Fast DDS* logging module allows for log entry filtering when consuming the logs, so that an application
execution output can be limited to specific areas of interest.
Beside the :ref:`dds_layer_log_verbosity_level`, *Fast DDS* provides three different filtering possibilities.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

It is worth mentioning that filters are applied in the specific order presented above, meaning that file name filtering
is only applied to the entries that pattern-match the category filter, and content filtering is only applied to the
entries that pattern-match both category and file name filters.


.. _dds_layer_log_filter_category:

Category Filtering
^^^^^^^^^^^^^^^^^^

Log entries can be filtered upon consumption according to their :ref:`dds_layer_log_category` component using regular
expressions.
Each time an entry is ready to be consumed, the category filter is applied using |std::regex_search-api|.
To set a category filter, member function |Log::SetCategoryFilter-api| is used:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_CATEGORY_FILTER
    :end-before: //!--
    :dedent: 4

The previous example would produce the following output:

.. code::

    2020-05-27 15:07:05.771 [CATEGORY_FILTER_1 Error] First log entry -> Function main
    2020-05-27 15:07:05.771 [CATEGORY_FILTER_2 Error] Second log entry -> Function main


.. _dds_layer_log_filter_filename:

File Name Filtering
^^^^^^^^^^^^^^^^^^^

Log entries can be filtered upon consumption according to their :ref:`dds_layer_log_file_context` component using
regular expressions.
Each time an entry is ready to be consumed, the file name filter is applied using |std::regex_search-api|.
To set a file name filter, member function |Log::SetFilenameFilter-api| is used:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_FILENAME_FILTER
    :end-before: //!--
    :dedent: 4

The previous example would produce the following output:

.. code::

    2020-05-27 15:07:05.771 [CATEGORY Error] First log entry (example.cpp:50) -> Function main

.. note::

    File name filters are applied even when the :ref:`dds_layer_log_file_context` entry component is disabled.


.. _dds_layer_log_filter_content:

Content Filtering
^^^^^^^^^^^^^^^^^

Log entries can be filtered upon consumption according to their :ref:`dds_layer_log_message` component using regular
expressions.
Each time an entry is ready to be consumed, the content filter is applied using |std::regex_search-api|.
To set a content filter, member function |Log::SetErrorStringFilter-api| is used:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //LOG_CONTENT_FILTER
    :end-before: //!--
    :dedent: 4

The previous example would produce the following output:

.. code::

    2020-05-27 15:07:05.771 [CATEGORY Error] First log entry -> Function main


Reset Logging Filters
^^^^^^^^^^^^^^^^^^^^^

The logging module's filters can be reset with member function |Log::Reset-api|.

.. warning::

    Resetting the module's filters entails:

    * Setting :ref:`dds_layer_log_verbosity_level` to |Log::Kind::Error-api|.
    * Disabling :ref:`dds_layer_log_file_context` component.
    * Enabling :ref:`dds_layer_log_function_name` component.
    * Clear all :ref:`dds_layer_log_filter`.
    * Clear all consumers and set a STDOUT or STDOUTERR consumer (depending on the CMake option |LOG_CONSUMER_DEFAULT|).
