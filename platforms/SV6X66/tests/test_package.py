import importlib.util
from pathlib import Path
import hashlib
import struct
import unittest
import zlib

spec = importlib.util.spec_from_file_location('sv_package', Path(__file__).parents[1] / 'package.py')
pkg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pkg)

class PackageTests(unittest.TestCase):
    def image(self):
        image = bytearray(pkg.APP_START + 128)
        struct.pack_into('<10I', image, 0, 0, 40, 80, 4, pkg.MAIN_SIZE, pkg.FLASH_SIZE, 0, 0, 1, pkg.RAW_SIZE)
        return image

    def test_envelope(self):
        image = self.image()
        framed = pkg.wrap_image(image)
        magic, version, length, app, fs, digest, reserved, crc = struct.unpack_from('<8sIIII16sII', framed)
        self.assertEqual((magic, version, length, app, fs, reserved),
                         (b'OBKSV616', 1, len(image), pkg.APP_START, pkg.FS_START, 0))
        self.assertEqual(digest, hashlib.md5(image).digest())
        self.assertEqual(crc, zlib.crc32(framed[:44]))
        self.assertEqual(framed[48:], image)

    def test_wrong_layout(self):
        for offset in (4, 8, 12, 16, 20, 24, 28, 36):
            image = self.image()
            image[offset] ^= 1
            with self.assertRaises(ValueError): pkg.wrap_image(image)

    def test_wrong_length(self):
        for length in (0, 40, pkg.APP_START, pkg.FS_START + 1, pkg.FLASH_SIZE):
            with self.assertRaises(ValueError): pkg.wrap_image(bytes(length))

if __name__ == '__main__':
    unittest.main()
