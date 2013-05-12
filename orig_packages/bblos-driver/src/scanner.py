#!/usr/bin/python

"""\
Autonomous motion with odometry loopback and scan retrieval
Copyright (C) 2008 BlueBotics SA
"""

import sys
import time
import Gnuplot


from Los.AntPlatform import *       # Contains gf* and pt* constants
from Los.Client import Connection
from Los.Types import *             # Contains all serializable types

    
def readScans(address, gnuplot):
    """Read scans from the platform."""
    proxy = Connection(address)
    proxy.login("User", "none")
    while True:
		(t, pose, maxAge, indices, coordinates) = proxy.Scan.get(Int32(gfCoordinates | gfSync))
		n = len(coordinates)/2
		print "%f: Received %d points, age %f" % (t,n,maxAge)
		print pose
		print maxAge
		print indices
		print coordinates
		sep = "\n"
		cb = [("%f %f" % (coordinates[2*i],coordinates[2*i+1])) for i in range(0,n)];
		# print sep.join(cb)
		gnuplot("set grid")
		gnuplot("set xrange [0.0:3.0]")
		gnuplot("set yrange [-1.5:1.5]")
		gnuplot.plot([(coordinates[2*i],coordinates[2*i+1]) for i in range(0,n)])

		time.sleep(0.1)


def main(argv):
	"""Launch localization and scan reader threads, and move along a node list."""
	address = ("10.0.1.10", 1234)     # Platform address
	gnuplot = Gnuplot.Gnuplot()

	try:
		readScans(address,gnuplot)
	except KeyboardInterrupt:
		print "Interrupted"


    
if __name__ == "__main__":
    sys.exit(main(sys.argv))
