Import linked libraries and its dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The DDS application requires the Fast DDS, Fast CDR and Fast DDS Python bindings libraries.
The way we will make these accessible from the
workspace depends on the installation procedure we have followed in the Installation Manual.

.. TODO Add when already python bindings are inside Fast-DDS
    Installation from binaries and manual installation
    """"""""""""""""""""""""""""""""""""""""""""""""""

    If we have followed the installation from binaries or the manual installation, these libraries are already
    accessible from the workspace.
    On Linux, the header files can be found in directories `/usr/include/fastrtps/` and
    `/usr/include/fastcdr/` for Fast DDS and Fast CDR respectively. The compiled libraries of both can be found in
    the directory `/usr/lib/`.

Colcon installation
"""""""""""""""""""

If you have followed the Colcon installation there are several ways to import the libraries.
If you want these to be accessible only from the current shell session, run one of the following two commands.

.. code-block:: bash

    source <path/to/Fast-DDS-python/workspace>/install/setup.bash

If you want these to be accessible from any session, you can add the Fast DDS installation directory to your ``$PATH``
variable in the shell configuration files running the following command.

.. code-block:: bash

    echo 'source <path/to/Fast-DDS-python/workspace>/install/setup.bash' >> ~/.bashrc

