# SPDX-License-Identifier: GPL-2.0
"""
Reject constant range bounds when the lower bound is greater than the upper.
"""

def test(conf):
    assert conf.olddefconfig() == 1
    assert conf.stderr_matches('expected_stderr')
