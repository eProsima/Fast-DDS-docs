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

    .. code-block:: bash

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
     - IDL grammar apply case sensitive matching.
   * - -d <directory>
     - Sets an output directory for generated files.
   * - -default-container-prealloc-size
     - Sets the default preallocated size for containers (sequence and maps). Default value: 0.
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
     - Ignore input files relative paths and place all generated files in the specified output directory.
   * - -fusion
     - Activates fusion.
   * - -help
     - Shows the help information
   * - -I <directory>
     - Add directory to preprocessor include paths.
   * - -language <lang>
     - chooses between <c++> or <java> languages.
   * - -no-typesupport
     - Avoid generating the type support files.
   * - -no-typeobjectsupport
     - Avoid generating the TypeObject support specific files. |br|
       Enabled automatically if -no-typesupport argument is used.
   * - -no-dependencies
     - Avoid processing the dependent IDL files.
   * - -package
     - Default package used in Java files.
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
   * - -typesc
     - Generates string and sequence types compatible with C.
   * - -version
     - Shows the current version of eProsima *Fast DDS-Gen*.

Please refer to :ref:`dynamic-types` for more information on TypeObject representation.
