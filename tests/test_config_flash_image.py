from pathlib import Path
import struct
import unittest

from tools.verify_config_fat import ConfigFatValidationError
from tools.verify_config_fat import calculate_runtime_sector_count
from tools.verify_config_fat import validate_config_fat_image


ROOT = Path(__file__).resolve().parents[1]
COMPONENT_CMAKE = (ROOT / "SRC/CMakeLists.txt").read_text(encoding="utf-8")
SDKCONFIG_DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
MANUFACTURING_FLASH = ROOT / "manufacturing_tools/flash_programmer.py"

CONFIG_PARTITION_SIZE = 0x400000
FAT_SECTOR_SIZE = 512
WL_FLASH_SECTOR_SIZE = 4096
EXPECTED_SAFE_SECTORS = 8080


def build_test_image(
    *,
    bytes_per_sector: int = FAT_SECTOR_SIZE,
    bpb_sectors: int = EXPECTED_SAFE_SECTORS,
    signature: bytes = b"\x55\xaa",
) -> bytes:
    image = bytearray(CONFIG_PARTITION_SIZE)
    boot_offset = WL_FLASH_SECTOR_SIZE
    struct.pack_into("<H", image, boot_offset + 11, bytes_per_sector)
    if bpb_sectors <= 0xFFFF:
        struct.pack_into("<H", image, boot_offset + 19, bpb_sectors)
    else:
        struct.pack_into("<I", image, boot_offset + 32, bpb_sectors)
    image[boot_offset + 510 : boot_offset + 512] = signature

    config_offset = CONFIG_PARTITION_SIZE - WL_FLASH_SECTOR_SIZE
    struct.pack_into(
        "<IIIIIIII",
        image,
        config_offset,
        0,
        CONFIG_PARTITION_SIZE,
        WL_FLASH_SECTOR_SIZE,
        WL_FLASH_SECTOR_SIZE,
        16,
        16,
        2,
        32,
    )
    return bytes(image)


class ConfigFlashImageTests(unittest.TestCase):
    def test_four_mib_safety_wl_exposes_8080_fat_sectors(self) -> None:
        self.assertEqual(
            calculate_runtime_sector_count(CONFIG_PARTITION_SIZE),
            EXPECTED_SAFE_SECTORS,
        )

    def test_matching_bpb_and_safety_wl_capacity_are_accepted(self) -> None:
        capacity = validate_config_fat_image(
            build_test_image(), CONFIG_PARTITION_SIZE
        )
        self.assertEqual(capacity.bytes_per_sector, FAT_SECTOR_SIZE)
        self.assertEqual(capacity.bpb_sectors, EXPECTED_SAFE_SECTORS)
        self.assertEqual(capacity.runtime_sectors, EXPECTED_SAFE_SECTORS)

    def test_legacy_performance_mode_bpb_capacity_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            ConfigFatValidationError,
            "actual=8096 expected=8080 sectors",
        ):
            validate_config_fat_image(
                build_test_image(bpb_sectors=8096), CONFIG_PARTITION_SIZE
            )

    def test_wrong_fat_sector_size_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            ConfigFatValidationError, "FAT sector size mismatch"
        ):
            validate_config_fat_image(
                build_test_image(bytes_per_sector=4096),
                CONFIG_PARTITION_SIZE,
            )

    def test_invalid_bpb_signature_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            ConfigFatValidationError, "BPB signature is invalid"
        ):
            validate_config_fat_image(
                build_test_image(signature=b"\x00\x00"),
                CONFIG_PARTITION_SIZE,
            )

    def test_wrong_image_size_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            ConfigFatValidationError, "image size mismatch"
        ):
            validate_config_fat_image(
                build_test_image()[:-1], CONFIG_PARTITION_SIZE
            )

    def test_config_flash_build_target_generates_and_validates_safe_wl(self) -> None:
        generation = COMPONENT_CMAKE.index("--wl_mode safe")
        validation = COMPONENT_CMAKE.index('"${config_fat_validator}"')
        dependency = COMPONENT_CMAKE.index(
            "add_dependencies(config-flash fatfs_config_bin)"
        )
        self.assertLess(generation, validation)
        self.assertLess(validation, dependency)
        self.assertIn("add_custom_target(\n    fatfs_config_bin ALL", COMPONENT_CMAKE)
        self.assertIn("--sector_size 512", COMPONENT_CMAKE)
        self.assertNotIn("fatfs_create_spiflash_image(config", COMPONENT_CMAKE)

    def test_build_configuration_requires_safety_wl(self) -> None:
        for setting in (
            "CONFIG_FATFS_SECTOR_512",
            "CONFIG_WL_SECTOR_SIZE_512",
            "CONFIG_WL_SECTOR_MODE_SAFE",
        ):
            with self.subTest(setting=setting):
                self.assertIn(f"NOT {setting}", COMPONENT_CMAKE)
                self.assertIn(f"{setting}=y", SDKCONFIG_DEFAULTS)

    @unittest.skipUnless(
        MANUFACTURING_FLASH.is_file(),
        "private manufacturing tool is not present in this checkout",
    )
    def test_local_manufacturing_generator_uses_same_wl_contract(self) -> None:
        source = MANUFACTURING_FLASH.read_text(encoding="utf-8")
        command = source[
            source.index("def _config_generator_command(") :
            source.index("def _require_safe_storage_configuration(")
        ]
        self.assertIn('"--sector_size",\n        "512"', command)
        self.assertIn('"--wl_mode",\n        "safe"', command)


if __name__ == "__main__":
    unittest.main()
