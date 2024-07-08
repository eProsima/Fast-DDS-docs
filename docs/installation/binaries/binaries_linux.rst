.. _linux_binaries:

Linux installation from binaries
================================

The instructions for installing *eProsima Fast DDS* in a Linux environment from
binaries are provided in this page.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _install_bl:

Install
-------

The latest release of *eProsima Fast DDS* for Linux is available at the eProsima website
`Downloads tab <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, extract the contents in your preferred directory.
Then, to install *eProsima Fast DDS* and all its dependencies in the system, execute
the :code:`install.sh` script with administrative privileges:

.. code-block:: bash

    cd <extraction_directory>
    sudo ./install.sh

.. note::

    By default, *eProsima Fast DDS* does not compile tests. To activate them, please refer to the :ref:`linux_sources`
    page.

.. _contents_bl:

Contents
^^^^^^^^

The :code:`src` folder contains the following packages:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library for data serialization according to the
  `CDR standard <https://www.omg.org/spec/DDSI-RTPS/2.2>`_ (*Section 10.2.1.2 OMG CDR*).
* :code:`fastdds`, the core library of *eProsima Fast DDS* library.
* :code:`fastddsgen`, a Java application that generates source code using the data types defined in an IDL file.

.. seealso::

    For further information about Fast DDS dependencies, as well as for the corresponding versions of other related
    products, please refer to the Fast DDS :ref:`dependencies_compatibilities` section.

In case any of these components is unwanted, it can be simply renamed or removed from the :code:`src`
directory.

.. _run_app_bl:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed, :code:`/usr/local/lib/`. There are two possibilities:

* Prepare the environment locally by typing in the console used for running the *eProsima Fast DDS* instance
  the command:

  .. code-block:: bash

      export LD_LIBRARY_PATH=/usr/local/lib/

* Add it permanently to the :code:`PATH` by executing:

  .. code-block:: bash

      echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc


.. _linking_bl:

Including Fast-DDS in a CMake project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The installer deploys *CMake config* files that simplify to incorporate **Fast-DDS** to any CMake project via
the *find_package* CMake API.

By setting the CMake variable **BUILD_SHARED_LIBS** is possible to choose the desired linkage (dynamic or static
library) in the CMake generator stage.
If the variable is missing build process will default to static linking.

For example in order to build the examples dynamically linked to **Fast-DDS** do:

   .. code-block:: bash

    $ cmake -Bbuildexample -DBUILD_SHARED_LIBS=ON .
    $ cmake --build buildexample --target install

.. _cli_bl:

Fast DDS CLI (optional)
-----------------------

The :ref:`Fast DDS CLI<ffastddscli_cli>` (Command Line Interface) is a tool that provides a set commands and
sub-commands to perform, Fast DDS related, maintenance and configuration tasks.
As an optional tool, its dependencies are not installed by default, but they can be installed by running the
following command:

.. code-block:: bash

    sudo apt-get install python3 python3-pip
    pip3 install xmlschema

Python3 is required to run the CLI tool, and the `xmlschema <https://pypi.org/project/xmlschema/>`_ dependency is
needed to use the :ref:`XML validation command<cli_xml>`.

.. _uninstall_bl:

Uninstall
---------

To uninstall all installed components, execute the `uninstall.sh` script (with administrative privileges):

.. code-block:: bash

    cd <extraction_directory>
    sudo ./uninstall.sh

.. warning::

    If any of the other components were already installed in some other way in the system, they will be
    removed as well. To avoid it, edit the script before executing it.
