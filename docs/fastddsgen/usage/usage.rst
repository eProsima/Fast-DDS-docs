.. include:: ../includes/aliases.rst

.. _fastddsgen_usage:

Usage
=====

This section explains the usage of *Fast DDS-Gen*  tool and briefly describes the generated files.

Running the *Fast DDS-Gen* Java application
----------------------------------------------

First, the steps outlined in :ref:`Linux installation of Fast DDS-Gen <fastddsgen_sl>` or
:ref:`Window installation of Fast DDS-Gen <fastddsgen_sw>` must be accomplished for the installation of *Fast DDS-Gen*.
According to this section, an executable file for Linux and Windows that runs the Java *Fast DDS-Gen* application is
available in the ``scripts`` folder.
If the ``scripts`` folder path is added to the ``PATH`` environment variable, *Fast DDS-Gen* can be executed running
the following commands:

- Linux:

  .. code-block:: bash

      $ fastddsgen

-  Windows:

   .. code-block:: winbatch

       > fastddsgen.bat

.. note::

    In case the PATH has not been modified, these scripts can be found in the ``<fastddsgen_directory>/scripts``
    directory.

.. _fastddsgen_supported_options:

Supported options
-------------------

The expected argument list of the application is:

.. code-block:: bash

    fastddsgen [<options>] <IDL file> [<IDL file> ...]

Where the options are:

.. list-table::
   :header-rows: 1
   :align: left

   * - Option
     - Description
   * - -cs
     - Enables case sensitivity in variable names.
   * - -d <directory>
     - Sets the output directory for the generated files.
   * - -default-container-prealloc-size
     - Sets the default preallocated size for unbounded collections (sequences and maps) |br|
       Default value: 0 (empty collection).
   * - -default_extensibility <extensibility> |br|
       -de <extensibility>
     - Sets the default extensibility for types without the @extensibility annotation. |br|
       Values: |br|
       - final |br|
       - appendable (default) |br|
       - mutable
   * - -example <platform>
     - Generates an example and a solution to compile the generated source code for a specific |br|
       platform.
       The help command shows the supported platforms.
   * - -extrastg <template> <output>
     - Specifies a custom template used for generating source code. |br|
       This option expects the location of the template and the location of the file where source code output will be
       stored. |br|
       A custom template example can be found in this `link <https://raw.githubusercontent.com/eProsima/Fast-DDS-Gen/master/resources/Custom.stg>`_
   * - -flat-output-dir
     - Ignores input files relative paths and place all generated files in the specified output directory.
   * - -help
     - Shows the help information
   * - -I <directory>
     - Adds directory to preprocessor include paths.
   * - -no-typesupport
     - Avoids generating the type support files.
   * - -no-typeobjectsupport
     - Avoids generating the TypeObject support specific files. |br|
       Enabled automatically if -no-typesupport argument is used.
   * - -no-dependencies
     - Avoids processing the dependent IDL files.
   * - -ppDisable
     - Disables the preprocessor.
   * - -ppPath
     - Specifies the preprocessor path.
   * - -python
     - Generates source code and a CMake solution to compile a library containing the data types |br|
       Python bindings required to run a *Fast DDS* Python-based application.
       This option is incompatible with `-example`.
   * - -replace
     - Replaces the generated source code files even if they exist.
   * - -t <directory>
     - Sets a specific directory as a temporary directory.
   * - -typeros2
     - Generates type naming compatible with ROS 2.
   * - -version
     - Shows the current version of eProsima *Fast DDS-Gen*.

Please refer to :ref:`dynamic-types` for more information on TypeObject representation.
