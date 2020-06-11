Building publisher/subscriber code
----------------------------------
This section guides you through the usage of this Java application and briefly describes the generated files.

Once you added ``scripts`` folder to your ``PATH``,
the Java application can be executed using the following scripts depending on if you are on Windows or Linux: ::

    > fastrtpsgen.bat
    $ fastrtpsgen

In case you didn't modified your ``PATH`` you can find these scripts in your ``<fastrtpsgen_directory>/scripts`` folder.

The expected argument list of the application is: ::

    fastrtpsgen [<options>] <IDL file> [<IDL file> ...]

Where the option choices are:

    +---------------------+-----------------------------------------------------------------------------------------+
    | Option              | Description                                                                             |
    +=====================+=========================================================================================+
    | -help               | Shows the help information.                                                             |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -version            | Shows the current version of eProsima Fast DDS-Gen.                                     |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -d <directory>      | Sets the output directory where the generated files are created.                        |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -I <directory>      | Add directory to preprocessor include paths.                                            |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -t <directory>      | Sets a specific directory as a temporary directory.                                     |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -example <platform> | Generates an example and a solution to compile the generated source code for a specific |
    |                     | platform. The help command shows the supported platforms.                               |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -replace            | Replaces the generated source code files even if they exist.                            |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -ppDisable          | Disables the preprocessor.                                                              |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -ppPath             | Specifies the preprocessor path.                                                        |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -typeobject         | Generates `TypeObject` files for the IDL provided and modifies MyType constructor to    |
    |                     | register the TypeObject representation into the factory.                                |
    +---------------------+-----------------------------------------------------------------------------------------+

For more information about TypeObject representation read :ref:`dynamic-types`.
