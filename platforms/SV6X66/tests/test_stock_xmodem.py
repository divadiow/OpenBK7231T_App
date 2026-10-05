import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stock_layout import stock_xmodem_image

START, END = 0xB000, 0xBA000

def receive(stream, block):
    # Factory B3E starts r9=0; D88 skips below start; DAC programs r9 directly.
    flash = bytearray(b'\xff' * END)
    written = 0
    for offset in range(0, len(stream), block):
        if START <= offset < END:
            data = stream[offset:min(offset + block, END)]
            flash[offset:offset + len(data)] = data
            written += len(data)
    return flash, written

class StockXmodemTests(unittest.TestCase):
    def test_raw_probe_is_not_programmed(self):
        raw = b'\x48\0\0\2' + b'P' * 4092
        for block in (128, 1024):
            self.assertEqual(receive(raw, block)[1], 0)

    def test_packaged_probe_round_trip(self):
        raw = b'\x48\0\0\2' + bytes(range(256)) * 15 + b'P' * 252
        stream = stock_xmodem_image(raw)
        self.assertEqual(stream[:START], b'\xff' * START)
        self.assertEqual(stream[START:], raw)
        for block in (128, 1024):
            flash, written = receive(stream, block)
            self.assertEqual(written, len(raw))
            self.assertEqual(flash[START:START + len(raw)], raw)
            self.assertEqual(flash[:START], b'\xff' * START)

    def test_partition_and_entry_checks(self):
        raw = b'\x48\0\0\2' + b'A' * (END - START - 4)
        self.assertEqual(len(stock_xmodem_image(raw)), END)
        for invalid in (b'', b'FAIL' + b'A' * 124, raw + b'A' * 128):
            with self.assertRaises(ValueError):
                stock_xmodem_image(invalid)

if __name__ == '__main__':
    unittest.main()
