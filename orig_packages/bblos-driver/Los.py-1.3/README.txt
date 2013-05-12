= Los.py =

[source:Lib/Los.py]

== Description ==
Los.py is the Python implementation of the LOS protocol.

== Requirements ==
 * Any platform able to run a stock Python.

== How to run ==
 * Los.py is a library, there is nothing to run here.

== Run-time dependencies ==
 * The Python programming language (>=Python-2.4.4)[[BR]]
   [http://www.python.org/]

== How to install ==
 * On POSIX platforms, run the following command in the root of the project:
{{{
python setup.py install
}}}
 * On Windows, run the .exe installer.

== Compile-time dependencies ==
 * There are none.
 
== How to compile ==
 * To generate a source distribution package, run the following command in the 
   root of the project:
{{{
python setup.py sdist
}}}
 * To generate a binary installer for Windows, run the following command in the
   root of the project:
{{{
python.exe setup.py bdist --formats=wininst
}}}
