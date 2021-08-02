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


Supported options
-------------------

The expected argument list of the application is:

.. code-block:: bash

    fastddsgen [<options>] <IDL file> [<IDL file> ...]

Where the option choices are:

+---------------------+------------------------------------------------------------------------------------------------+
| Option              | Description                                                                                    |
+=====================+================================================================================================+
| -help               | Shows the help information.                                                                    |
+---------------------+------------------------------------------------------------------------------------------------+
| -version            | Shows the current version of eProsima *Fast DDS-Gen*.                                          |
+---------------------+------------------------------------------------------------------------------------------------+
| -d <directory>      | Sets the output directory where the generated files are created.                               |
+---------------------+------------------------------------------------------------------------------------------------+
| -I <directory>      | Add directory to preprocessor include paths.                                                   |
+---------------------+------------------------------------------------------------------------------------------------+
| -t <directory>      | Sets a specific directory as a temporary directory.                                            |
+---------------------+------------------------------------------------------------------------------------------------+
| -example <platform> | Generates an example and a solution to compile the generated source code for a specific |br|   |
|                     | platform. The help command shows the supported platforms.                                      |
+---------------------+------------------------------------------------------------------------------------------------+
| -replace            | Replaces the generated source code files even if they exist.                                   |
+---------------------+------------------------------------------------------------------------------------------------+
| -ppDisable          | Disables the preprocessor.                                                                     |
+---------------------+------------------------------------------------------------------------------------------------+
| -ppPath             | Specifies the preprocessor path.                                                               |
+---------------------+------------------------------------------------------------------------------------------------+
| -typeobject         | Generates `TypeObject` files for the IDL provided and modifies MyType constructor to |br|      |
|                     | register the TypeObject representation into the factory.                                       |
+---------------------+------------------------------------------------------------------------------------------------+
| -typeros2           | Generates type naming compatible with ROS 2                                                    |
+---------------------+------------------------------------------------------------------------------------------------+
| -cs                 | Activates Case Sensitive                                                                       |
+---------------------+------------------------------------------------------------------------------------------------+

Please refer to :ref:`dynamic-types` for more information on TypeObject representation.
