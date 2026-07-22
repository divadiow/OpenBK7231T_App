from __future__ import annotations

import importlib.util
import json
import os
import struct
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "txw81x_pack.py"
SPEC = importlib.util.spec_from_file_location("txw81x_pack", MODULE_PATH)
assert SPEC and SPEC.loader
pack = importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name] = pack
SPEC.loader.exec_module(pack)

MANIFEST = json.loads((Path(__file__).with_name("reference_manifest.json")).read_text())


def txw81x_config() -> pack.ImageConfig:
    sequences = tuple(
        pack._parse_spi_sequence(value)
        for value in ("06000000", "0100020002", "31000102", "05800101")
    )
    return pack.ImageConfig(
        boot_flag=0x5A69,
        version=0,
        load_address=0x18000000,
        execute_address=0x18000000,
        code_offset=0xC00,
        flash_size=0x100000,
        spi_clock_mhz=0x3C,
        driver_strength=0,
        pll_source_mhz=0x28,
        pll_enabled=True,
        debug_enabled=False,
        aes_enabled=False,
        code_crc16_enabled=True,
        read_command=0xEB,
        read_dummy_cycles=6,
        clock_mode=0,
        sample_delay=0x55AA,
        command_wires=1,
        address_wires=4,
        data_wires=4,
        quad_select=0,
        quad_enabled=True,
        special_sequence_enabled=True,
        special_sequences=sequences,
        chip_id=0x8410,
        cpu_id=0,
        customer_id=0x1001,
    )


class AlgorithmTests(unittest.TestCase):
    def test_crc_vectors(self) -> None:
        data = b"123456789"
        self.assertEqual(pack.crc16_modbus(data), 0x4B37)
        self.assertEqual(pack.crc32_hgic(data), 0xCBF43926)

    def test_spi_zero_data_sequence_ignores_ini_padding_byte(self) -> None:
        self.assertEqual(pack._parse_spi_sequence("06000000"), bytes.fromhex("060000"))

    def test_intel_hex_remap_and_gap_fill(self) -> None:
        # Two records at 0x18000000 and 0x18000010 with an eight-byte gap.
        text = "\n".join(
            (
                ":020000041800E2",
                ":080000000102030405060708D4",
                ":04001000090A0B0CC2",
                ":00000001FF",
            )
        )
        base, binary = pack.ihex_to_binary(text)
        self.assertEqual(base, 0x18000000)
        self.assertEqual(binary[:8], bytes(range(1, 9)))
        self.assertEqual(binary[8:16], b"\xFF" * 8)
        self.assertEqual(binary[16:], bytes.fromhex("090A0B0C"))

    def test_parameter_extraction(self) -> None:
        code = bytearray(0x300)
        struct.pack_into("<H", code, 0x180, 0x20)
        source = bytearray(0x210 + 0x20)
        source[0x210:] = bytes(range(0x20))
        parameter = pack.extract_parameter(bytes(code), bytes(source), length_offset=0x180)
        self.assertEqual(len(parameter), 0x20)
        self.assertEqual(parameter[:2], b"\x20\x00")
        self.assertEqual(parameter[2:], bytes(range(2, 0x20)))

    def test_synthetic_app_is_deterministic_and_self_consistent(self) -> None:
        code = bytearray((index * 17 + 3) & 0xFF for index in range(0x700))
        struct.pack_into("<H", code, 0x180, 0x40)
        parameter = bytearray(0x40)
        struct.pack_into("<H", parameter, 0, len(parameter))
        parameter[2:] = bytes((index * 11) & 0xFF for index in range(len(parameter) - 2))
        version = pack.VersionInfo(0x02050307, 0x1234, 0x07EA0716, 0)
        first = pack.build_app_image(bytes(code), bytes(parameter), txw81x_config(), version)
        second = pack.build_app_image(bytes(code), bytes(parameter), txw81x_config(), version)
        self.assertEqual(first, second)
        info = pack.parse_app_image(first)
        self.assertTrue(all(info["checks"].values()))


class KnownGoodTaixinTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        reference_dir = os.environ.get("TXW81X_REFERENCE_DIR")
        loader_path = os.environ.get("TXW81X_LOADER")
        if not reference_dir or not loader_path:
            raise unittest.SkipTest(
                "set TXW81X_REFERENCE_DIR and TXW81X_LOADER to run known-good Taixin integration tests"
            )
        cls.reference_dir = Path(reference_dir)
        cls.loader_path = Path(loader_path)
        cls.app_path = cls.reference_dir / MANIFEST["app"]["filename"]
        cls.ota_path = cls.reference_dir / MANIFEST["ota"]["filename"]
        for path in (cls.app_path, cls.ota_path, cls.loader_path):
            if not path.is_file():
                raise unittest.SkipTest(f"reference file is missing: {path}")

    def test_reference_hashes_and_parsed_fields(self) -> None:
        app = self.app_path.read_bytes()
        ota = self.ota_path.read_bytes()
        loader = self.loader_path.read_bytes()
        self.assertEqual(pack.hashlib.sha256(app).hexdigest(), MANIFEST["app"]["sha256"])
        self.assertEqual(pack.hashlib.sha256(ota).hexdigest(), MANIFEST["ota"]["sha256"])
        self.assertEqual(pack.hashlib.sha256(loader).hexdigest(), MANIFEST["loader"]["sha256"])

        app_info = pack.parse_app_image(app)
        expected_app = MANIFEST["app"]
        self.assertEqual(app_info["size"], expected_app["size"])
        self.assertEqual(app_info["boot"]["code_offset"], expected_app["code_offset"])
        self.assertEqual(app_info["boot"]["code_length"], expected_app["code_length"])
        self.assertEqual(app_info["boot"]["code_crc16"], expected_app["code_crc16"])
        self.assertEqual(app_info["parameter_offset"], expected_app["parameter_offset"])
        self.assertEqual(app_info["parameter_length"], expected_app["parameter_length"])
        self.assertEqual(app_info["firmware"]["parameter_crc16"], expected_app["parameter_crc16"])
        self.assertEqual(app_info["firmware"]["sdk_version"], expected_app["sdk_version"])
        self.assertEqual(app_info["firmware"]["svn_version"], expected_app["svn_version"])
        self.assertEqual(app_info["firmware"]["date"], expected_app["date"])
        self.assertEqual(app_info["firmware"]["chip_id"], expected_app["chip_id"])
        self.assertEqual(app_info["firmware"]["cpu_id"], expected_app["cpu_id"])
        self.assertTrue(all(app_info["checks"].values()))

        ota_info = pack.parse_ota_image(ota)
        expected_ota = MANIFEST["ota"]
        self.assertEqual(ota_info["size"], expected_ota["size"])
        self.assertEqual(ota_info["loader_length"], expected_ota["loader_length"])
        self.assertEqual(ota_info["compressed_image"]["size"], expected_ota["compressed_image_length"])
        self.assertEqual(
            ota_info["compressed_image"]["boot"]["code_length"], expected_ota["compressed_code_length"]
        )
        self.assertEqual(ota_info["compression"]["xz_length"], expected_ota["xz_length"])
        self.assertEqual(ota_info["compression"]["raw_length"], expected_ota["real_code_length"])
        self.assertEqual(ota_info["compression"]["raw_crc16"], expected_ota["real_code_crc16"])
        self.assertEqual(ota_info["compression"]["xz_crc16"], expected_ota["xz_crc16"])
        self.assertTrue(all(ota_info["checks"].values()))

    def test_app_repack_is_byte_identical(self) -> None:
        app = self.app_path.read_bytes()
        info = pack.parse_app_image(app)
        code_offset = info["boot"]["code_offset"]
        parameter_offset = info["parameter_offset"]
        parameter_length = info["parameter_length"]
        code = app[code_offset:]
        parameter = app[parameter_offset : parameter_offset + parameter_length]
        version = pack.VersionInfo(
            info["firmware"]["sdk_version"],
            info["firmware"]["svn_version"],
            info["firmware"]["date"],
            info["application_version"],
        )
        rebuilt = pack.build_app_image(code, parameter, txw81x_config(), version)
        self.assertEqual(rebuilt, app)

    def test_ota_repack_is_byte_identical(self) -> None:
        app = self.app_path.read_bytes()
        expected = self.ota_path.read_bytes()
        loader = self.loader_path.read_bytes()
        rebuilt = pack.build_ota_image(app, loader)
        self.assertEqual(rebuilt, expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
