import hashlib
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stock_phy import phy_overlay
app = Path(__file__).resolve().parents[3]
source = app / 'sdk/OpenSV6X66/platform/mcu/sv6266/sdk/components/drv/phy/libphy.a'
class StockPhyTests(unittest.TestCase):
    def test_only_two_instruction_spans_change(self):
        original = source.read_bytes()
        adapted, info = phy_overlay(original)
        self.assertEqual(len(original), len(adapted))
        self.assertTrue(original != adapted, 'clock-load instructions must change')
        self.assertEqual(len(info['archive_offsets']), 2)
        allowed = set()
        for offset, replacement in zip(info['archive_offsets'], ['440000199200', '440000509200']):
            self.assertEqual(adapted[offset:offset+6], bytes.fromhex(replacement))
            allowed.update(range(offset, offset+6))
        changed = {i for i, (a, b) in enumerate(zip(original, adapted)) if a != b}
        self.assertTrue(changed and changed <= allowed)
        self.assertEqual(hashlib.sha256(original).hexdigest(), info['original_sha256'])
        self.assertEqual(hashlib.sha256(adapted).hexdigest(), info['adapted_sha256'])
    def test_modified_or_already_adapted_archive_rejected(self):
        original = source.read_bytes()
        adapted, _ = phy_overlay(original)
        for bad in [adapted, original[:-1], b'wrong' + original[5:]]:
            with self.assertRaisesRegex(ValueError, 'SDK PHY archive'):
                phy_overlay(bad)
if __name__ == '__main__': unittest.main()
