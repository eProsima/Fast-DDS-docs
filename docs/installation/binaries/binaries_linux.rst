.. _linux_binaries:

Linux installation from binaries
================================

In this page, we provide the instructions for installing *eProsima Fast DDS* in a Linux environment from
binaries. It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _install_bl:

Install
-------

The latest release of *eProsima Fast DDS* for Linux is available at the company website
`downloads page <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, extract the contents of the package.

Now, to install *eProsima Fast DDS* and all its dependencies in the system, execute
the :code:`install.sh` script with administrative privileges:

.. code-block:: bash

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
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.
* :code:`fastrtpsgen`, a Java application that generates source code using the data types defined in an IDL file.

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

* Add it permanently to the :code:`PATH`, by typing:

  .. code-block:: bash

      echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc

.. _uninstall_bl:

Uninstall
---------

To uninstall all installed components, execute the :code:`uninstall.sh` script (with administrative privileges):

.. code-block:: bash

    sudo ./uninstall.sh

.. caution::

    If any of the other components were already installed in some other way in the system, they will be
    removed as well. To avoid it, edit the script before executing it.
