import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mahi/gps_frontier_explorer/exploring_exploration/install/gps_frontier_explorer'
