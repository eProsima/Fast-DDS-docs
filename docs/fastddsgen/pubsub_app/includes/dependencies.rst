Import linked libraries and its dependencies
----------------------------------------------

The DDS application requires the *Fast DDS* and *Fast CDR* libraries.
The way of making these accessible from the
workspace depends on the installation procedure followed in the Installation Manual.

Installation from binaries and manual installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the installation from binaries or the manual installation has been followed, these libraries are already accessible
from the workspace.
On Linux, the header files can be found in directories `/usr/include/fastrtps/` and
`/usr/include/fastcdr/` for *Fast DDS* and *Fast CDR* respectively.
The compiled libraries of both can be found in the directory `/usr/lib/`.

Colcon installation
^^^^^^^^^^^^^^^^^^^^

If the Colcon installation has been followed, there are several ways to import the libraries.
To make these accessible only from the current shell session, run one of the following two commands.

.. code-block:: bash

    source <path/to/Fast-DDS/workspace>/install/setup.bash

However, to make these accessible from any session, add the *Fast DDS* installation directory to the ``$PATH``
variable in the shell configuration files running the following command.

.. code-block:: bash

    echo 'source <path/to/Fast-DDS/workspace>/install/setup.bash' >> ~/.bashrc
