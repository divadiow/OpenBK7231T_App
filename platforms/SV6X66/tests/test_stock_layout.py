import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stock_layout import uart_image, linker_overlay, APP_XIP, FS_XIP


def fixture(data_address=APP_XIP + 0x100, startup=b'\x48\0\0\2'):
    names = b'\0.magic_boot\0.text\0.data\0.shstrtab\0'
    sections = [(1, APP_XIP, startup), (13, APP_XIP + len(startup), b'CODE'), (19, data_address, b'DATA')]
    elf = bytearray(0x500)
    elf[:16] = b'\x7fELF\1\1\1' + bytes(9)
    struct.pack_into('<HHIIIIIHHHHHH', elf, 16, 2, 167, 1, 0, 52, 0x400, 0, 52, 32, 3, 40, 5, 4)
    for i, (name, address, content) in enumerate(sections):
        offset = 0x200 + i * 0x40
        struct.pack_into('<8I', elf, 52 + i * 32, 1, offset, address, address, len(content), len(content), 5, 4)
        struct.pack_into('<10I', elf, 0x400 + (i + 1) * 40, name, 1, 6, address, offset, len(content), 0, 0, 4, 0)
        elf[offset:offset + len(content)] = content
    elf[0x300:0x300 + len(names)] = names
    struct.pack_into('<10I', elf, 0x400 + 4 * 40, 25, 3, 0, 0, 0x300, len(names), 0, 0, 1, 0)
    return bytes(elf)


class StockLayoutTests(unittest.TestCase):
    def test_uart_offset_zero_is_stock_startup(self):
        payload, sections = uart_image(fixture())
        self.assertEqual(payload[:8], b'\x48\0\0\2CODE')
        self.assertEqual(payload[0x100:], b'DATA')
        self.assertEqual(len(payload), 0x104)
        self.assertEqual(sections['.magic_boot']['lma'], APP_XIP)

    def test_rejects_preserved_region_and_partition_overflow(self):
        for address in (0x30008000, FS_XIP, APP_XIP + 2):
            with self.assertRaises(ValueError):
                uart_image(fixture(address))

    def test_rejects_truncated_or_wrong_elf(self):
        for elf in (fixture()[:60], bytes(100), fixture().replace(b'\x48\0\0\2', b'FAIL')):
            with self.assertRaises(ValueError):
                uart_image(elf)

    def test_rejects_vendor_pre_ilm_store(self):
        prefix = bytes.fromhex('4800000247d0010059de80006400800246101000402004024e2200084600000058000000420e002147f0012059ff8000')
        self.assertEqual(len(prefix), 0x30)
        with self.assertRaisesRegex(ValueError, 'pre-ILM'):
            uart_image(fixture(startup=prefix + bytes.fromhex('b620840f')))

    def test_sdk_overlay_preserves_ram_bookkeeping(self):
        app = Path(__file__).resolve().parents[3]
        source = (app / 'sdk/OpenSV6X66/platform/mcu/sv6266/sdk/projects/lite_mac/ld/lite_ilm_flash.lds.S').read_text()
        result = linker_overlay(source)
        self.assertIn('.magic_boot 0x3000B000 :', result)
        self.assertNotIn('ORIGIN(REGION_BURN) + M_PARAM_SECTOR_SIZE', result)
        self.assertLess(result.index('dlm_remain ='), result.index('.fix_table_section'))
        self.assertIn('. = M_FLASH_SECTOR_SIZE;', result)
        self.assertIn('__stock_app_end <= 0x300BA000', result)

if __name__ == '__main__':
    unittest.main()
