.. _windows_binaries:

Windows installation from binaries
==================================

You can download the latest release of *eProsima Fast DDS* for Windows from the company website
`downloads page <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, execute the installer and follow the instructions, choosing your preferred Visual Studio
version and architecture when prompted.

Requirements
------------

Visual Studio
^^^^^^^^^^^^^

To install *eProsima Fast DDS*, you need to have `Visual Studio <https://visualstudio.microsoft.com/>`_ installed.
*eProsima Fast RTPS* requires the Visual C++ Redistributable packages for the Visual Studio version you choose during
the installation or compilation. The installer gives you the option of downloading and installing them.

If you have Visual Studio but you haven’t the Visual C++ Redistributable packages installed,
open Visual Studio and perform the following steps:

* Go to :code:`Tools` -> :code:`Get Tools and Features`
* In the :code:`Workloads` tab enable :code:`Desktop development with C++`
* Click :code:`Modify` at the bottom right

TinyXML2
^^^^^^^^

You can install the TinyXML2 library using Chocolatey_. First, download the
following Chocolatey package from this
`ROS2 Github repository <https://github.com/ros2/choco-packages/releases/tag/2020-02-24>`_.

* tinyxml2.6.0.0.nupkg

Once this package is downloaded, open an administrative shell and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder you downloaded the packages into.

OpenSSL
^^^^^^^

You can download OpenSSL 1.0.2 for Windows at this webpage_.
This is the OpenSSL version tested by our team.
Download and use the installer that fits your requirements.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\OpenSSL-Win64

Packages
--------

By default, it will download all the available packages, namely:

- :code:`fastcdr`
- :code:`fastrtps`
- :code:`fastrtpsgen`
- :code:`foonathan-memory-vendor`

If you don't want to install some of these components, you can simply uncheck them during the installation process.

Environmental Variables
-----------------------

eProsima Fast RTPS requires the following environmental variable setup in order to function properly:

* FASTRTPSHOME: Root folder where eProsima Fast RTPS is installed.
* Additions to the PATH: the /bin folder and the subfolder for your Visual Studio version of choice should be appended
  to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

.. External links

.. _Chocolatey: https://chocolatey.org/
.. _webpage: https://slproweb.com/products/Win32OpenSSL.html
