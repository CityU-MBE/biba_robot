#!/usr/bin/env python
"""\
Installation script for Los.py
Copyright (C) 2006 BlueBotics SA
"""

from distutils.core import setup
from os.path import abspath, dirname, join
import sys


if __name__ == "__main__":
    if sys.version_info[:2] < (2, 5):
        print "This package requires version 2.5 or later of Python"
        sys.exit(1)
    
    # Import project metadata
    sys.path.insert(0, abspath(join(dirname(__file__), "lib")))
    from Los import Meta
    
    setup(
        name = Meta.project,
        version = Meta.version,
        author = Meta.author,
        author_email = Meta.authorEmail,
        license = Meta.license,
        url = Meta.url,
        download_url = Meta.download,
        description = Meta.description,
        long_description = Meta.longDescription,
        keywords = Meta.keywords,
        platforms = Meta.platforms,
        classifiers = Meta.classifiers,
        
        package_dir = {"": "lib"},
        packages = ["Los"],
        py_modules = [],
        scripts = [],
        data_files = [],
    )

