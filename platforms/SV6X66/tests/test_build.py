#!/usr/bin/env python3
"""Validate build guards without invoking a toolchain or mutating sources."""
from pathlib import Path
import subprocess
import sys
import unittest

app = Path(__file__).resolve().parents[3]
builder = app / 'platforms/SV6X66/build.py'

class BuildGuards(unittest.TestCase):
    def reject(self, arguments, message):
        result = subprocess.run([sys.executable, str(builder), '--toolchain', '/missing-toolchain'] + arguments,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn(message, result.stderr)

    def test_versions_cannot_escape_output(self):
        for version in ('.', '..', '../escape', 'path/name'):
            self.reject(['--version', version], 'version must contain')

    def test_sdk_staging_cannot_replace_sources(self):
        sdk = app / 'sdk/OpenSV6X66/platform/mcu/sv6266/sdk'
        for directory in (app, app.parent, sdk, sdk / 'nested', sdk.parent, app / 'sdk/OpenSV6X66'):
            self.reject(['--build-dir', str(directory)], 'must not replace')

    def test_staging_cannot_recurse_into_overlay_sources(self):
        for name in ('src', 'include', 'platforms', 'libraries'):
            for directory in (app / name, app / name / 'nested-stage'):
                self.reject(['--build-dir', str(directory)], 'must not replace')

    def test_jobs_must_be_positive(self):
        self.reject(['--jobs', '0'], 'jobs must be positive')

if __name__ == '__main__':
    unittest.main()
