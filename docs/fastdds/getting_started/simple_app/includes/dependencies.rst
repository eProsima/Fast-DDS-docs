Import linked libraries and its dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The DDS application requires the Fast-RTPS and Fast-CDR libraries.
The way we will make these accessible from the
workspace depends on the installation procedure we have followed in :ref:`installation-from-sources`.

Manual installation
"""""""""""""""""""

If we have followed a manual installation, these libraries are already accessible from the workspace.
On Linux, the header files can be found in directories `/usr/local/include/fastrtps/` and
`/usr/local/include/fastcdr/` for Fast RTPS and Fast CDR respectively. The compiled libraries of both can be found in
the directory `/usr/local/lib/`.

Colcon installation
"""""""""""""""""""

If you have followed the Colcon installation there are several ways to import the libraries.
If you want these to be accessible only from the current shell session, run one of the following two commands.

.. code-block:: bash

    source <path/to/Fast-RTPS/workspace>/install/setup.bash

If you want these to be accessible from any session, you can add the Fast-RTPS installation directory to your ``$PATH``
variable in the shell configuration files running the following command.

.. code-block:: bash

    echo 'source <path/to/Fast-RTPS/workspace>/install/setup.bash' >> ~/.bashrc
