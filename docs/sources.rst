Installation from Sources
=========================

Clone the project from Github: ::

    $ git clone https://github.com/eProsima/Fast-RTPS
    $ mkdir Fast-RTPS/build && cd Fast-RTPS/build

If you are on Linux, execute: ::

    $ cmake -DTHIRDPARTY=ON ..
    $ make
    $ sudo make install

If you are on Windows, choose your version of Visual Studio: ::

    > cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON ..
    > cmake --build . --target install

If you want to compile the performance tests, you will need to add the argument ``-DPERFORMANCE_TESTS=ON`` when calling Cmake.
