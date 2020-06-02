.. _installation_binaries_linux:

Linux
-----

Extract the contents of the package.
It will contain both *eProsima Fast DDS* and its required package *eProsima Fast CDR*.
You will have to follow the same procedure for both packages, starting with *Fast CDR*.

Configure the compilation: ::

        $ ./configure --libdir=/usr/lib

If you want to compile with debug symbols (which also enables verbose mode): ::

        $ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib

After configuring the project compile and install the library: ::

        $ sudo make install
