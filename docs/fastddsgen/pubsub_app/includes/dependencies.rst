Import linked libraries and its dependencies
--------------------------------------------

The DDS application requires the *Fast DDS* and *Fast CDR* libraries.
The way of making these accessible from the
workspace depends on the installation procedure followed in the Installation Manual.

Installation from binaries
^^^^^^^^^^^^^^^^^^^^^^^^^^

If the installation from binaries has been followed, these libraries are already accessible from the workspace.

*   On Linux: The header files can be found in directories ``/usr/include/fastrtps/`` and
    ``/usr/include/fastcdr/`` for *Fast DDS* and *Fast CDR* respectively.
    The compiled libraries of both can be found in the directory ``/usr/lib/``.
*   On Windows: The header files can be found in directories
    ``C:\Program Files\eProsima\fastrtps 2.0.0\include\fastrtps`` and
    ``C:\Program Files\eProsima\fastrtps 2.0.0\include\fastcdr\`` for *Fast DDS* and *Fast CDR* respectively.
    The compiled libraries of both can be found in the directory ``C:\Program Files\eProsima\fastrtps 2.0.0\lib\``.

Colcon installation
^^^^^^^^^^^^^^^^^^^

If the Colcon installation has been followed, there are several ways to import the libraries.
To make these accessible only from the current shell session, run one of the following two commands.

* On Linux:

.. code-block:: bash

    source <path/to/Fast-DDS/workspace>/install/setup.bash

* On Windows:

.. code-block:: bash

    <path/to/Fast-DDS/workspace>/install/setup.bat

However, to make these accessible from any session, add the *Fast DDS* installation directory to the ``$PATH``
variable in the shell configuration files running the following command.

* On Linux:

.. code-block:: bash

    echo 'source <path/to/Fast-DDS/workspace>/install/setup.bash' >> ~/.bashrc

* On Windows: Open the `Edit the system environment variables` control panel and add
  ``<path/to/Fast-DDS/workspace>/install/setup.bat`` to the ``PATH``.

