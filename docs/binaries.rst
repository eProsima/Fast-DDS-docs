Installation from Binaries
==========================

You can always download the latest binary release of *eProsima Fast RTPS* from the `company website <http://www.eprosima.com/>`_.

Windows 7 32-bit and 64-bit
---------------------------

Execute the installer and follow the instructions, choosing your preferred Visual Studio version and architecture when prompted.

Environmental Variables
^^^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* FASTRTPSHOME: Root folder where *eProsima Fast RTPS* is installed.
* Additions to the PATH: the /bin folder and the subfolder for your Visual Studio version of choice should be appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

Linux
-----

Extract the contents of the package. It will contain both *eProsima Fast RTPS* and its required package *eProsima Fast CDR*. You will have follow the same procedure for both packages, starting with *Fast CDR*.

Configure the compilation: ::

        $ ./configure --libdir=/usr/lib

If you want to compile with debug symbols (which also enables verbose mode): ::

        $ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib

After configuring the project compile and install the library: ::

        $ sudo make install
