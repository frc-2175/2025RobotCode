import testutil
from pyfrc.tests import *

import utils

# utils.clamp
testutil.expecteq(utils.clamp(0, 1, 10), 1)
testutil.expecteq(utils.clamp(1, 1, 10), 1)
testutil.expecteq(utils.clamp(5, 1, 10), 5)
testutil.expecteq(utils.clamp(10, 1, 10), 10)
testutil.expecteq(utils.clamp(11, 1, 10), 10)
testutil.expecterror(lambda: utils.clamp(5, 10, 1)) # min and max reversed
