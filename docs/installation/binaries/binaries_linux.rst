.. _linux_binaries:

Linux installation from binaries
================================

You can download the latest release of *eProsima Fast DDS* for Linux from the company website
`downloads page <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, extract the contents of the package.

Packages
--------

In the :code:`src` folder you will find:

- :code:`fastcdr`
- :code:`fastrtps`
- :code:`fastrtpsgen`
- :code:`foonathan_memory_vendor`

If you don't want to install any of these components, you can simply remove or rename its folder from the :code:`src`
directory.

Installing and uninstalling
---------------------------

To install *eProsima Fast DDS* and all its dependencies in the system, you have to run (with administrative privileges)
the :code:`install.sh` script:

.. code-block:: bash

    sudo ./install.sh

.. note::

    When running an *eProsima Fast DDS* application, you need to link it with the library where the packages have been
    installed. You can either prepare the environment locally by typing the command:

    .. code-block:: bash

        export LD_LIBRARY_PATH=/usr/local/lib/

    in the console you use to run the *eProsima Fast DDS* instance, or permanently add it to your path, by typing:

    .. code-block:: bash

        echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc


To uninstall all installed components, execute the :code:`uninstall.sh` script (with administrative privileges):

.. code-block:: bash

    sudo ./uninstall.sh

.. caution::

    If any of the other components were already installed in some other way in the system, they will be
    removed as well. To avoid it, edit the script before executing it.

.. note::

    By default, *eProsima Fast DDS* doesn’t compile tests.
    You can activate them by downloading and installing `Gtest <https://github.com/google/googletest>`_.
