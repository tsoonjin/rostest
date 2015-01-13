#!/usr/bin/env python

import sys
import importlib

'''
Bootstrap script to run vision module and submodule
'''

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print "Loading module..."
        mod = importlib.import_module(sys.argv[1])
        print "Module loaded"

        if not mod:
            print "Cannot import module: %s" % sys.argv[1]
        else:
            main = mod.main
            if not main:
                print "Module must have a main() function"
            else:
                print "Executed"
                main()
    else:
        print "Usage: run [packages.]module_name [arguments]"
