"""\
Constants for ANT platform
Copyright (C) 2008 BlueBotics SA
"""

# Scan.get() flags
gfCoordinates   = 1 << 0
gfCoordinates3D = 1 << 1
gfAges          = 1 << 2
gfIntensities   = 1 << 3
gfTypes         = 1 << 4
gfSync          = 1 << 8
gfSyncMem       = 1 << 9
gfAsync         = 1 << 10
gfAsyncMem      = 1 << 11

# Scan point types
ptSynchronous = 0
ptExternal = 1
ptVirtual = 2
ptUltrasound = 3
ptInfrared = 4
