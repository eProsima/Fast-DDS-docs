Execution and IDL Definition
============================

Building publisher/subscriber code
----------------------------------
This section guides you through the usage of this Java application and briefly describes the generated files.

The Java application can be executed using the following scripts depending on if you are on Windows or Linux: ::

	> fastrtpsgen.bat ::
 hghgtrtpsgen

The expected argument list of the application is: ::

	fastrtpsgen [<options>] <IDL file> [<IDL file> ...]

Where the option choices are:

+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Option              | Description																	 |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| -help               | Shows the help information.															 |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| -version            | Shows the current version of eProsima FASTRTPSGEN.												 |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| -d <directory>      | Output directory where the generated files are created.												 |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| -example <platform> |Generates an example and a solution to compile the generated source code for a specific platform. The help command shows the supported platforms. |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| -replace            |Replaces the generated source code files whether they exist.											 |
+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+





Defining a data type via IDL
----------------------------
