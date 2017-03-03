.. _installation-from-sources:

Installation from Sources
=========================

Clone the project from Github: ::

    $ git clone https://github.com/eProsima/Fast-RTPS

If you are on Linux, execute: ::

    $ cmake ../ -DEPROSIMA_BUILD=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install
    $ make
    $ make install 

If you are on Windows, choose your version of Visual Studio: ::

    > cmake ../  -G"Visual Studio 14 2015 Win64" -DEPROSIMA_BUILD=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=installationpath
    > cmake --build . --target install
	
If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling Cmake.
