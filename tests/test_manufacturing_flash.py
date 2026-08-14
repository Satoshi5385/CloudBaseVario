from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from manufacturing_tools.flash_programmer import (
    BuildArtifacts,
    EXPECTED_BOARD_DATA_OFFSET,
    EXPECTED_CONFIG_OFFSET,
    FlashProgrammer,
    _config_generator_command,
    _image_is_fully_erased,
)


class ManufacturingFlashTests(unittest.TestCase):
    def _artifacts(self, root: Path) -> tuple[BuildArtifacts, dict[str, Path]]:
        names = (
            "bootloader.bin",
            "partition-table.bin",
            "phy.bin",
            "app.bin",
            "otadata.bin",
            "config.bin",
            "board_data.bin",
        )
        files = {name: root / name for name in names}
        for path in files.values():
            path.write_bytes(b"test")
        artifacts = BuildArtifacts(
            build_dir=root,
            python_executable=Path("python"),
            idf_path=root,
            project="CloudBaseVario-Aohazuku",
            version="test",
            application=files["app.bin"],
            application_sha256="a" * 64,
            write_flash_args=("--flash-size", "16MB"),
            flash_files=(
                (0, files["bootloader.bin"]),
                (0x8000, files["partition-table.bin"]),
                (0xF000, files["phy.bin"]),
                (0x10000, files["app.bin"]),
                (EXPECTED_CONFIG_OFFSET, files["config.bin"]),
                (0x810000, files["otadata.bin"]),
            ),
            config_image=files["config.bin"],
        )
        return artifacts, files

    def test_verification_set_contains_config_and_per_device_nvs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifacts, files = self._artifacts(root)
            arguments = FlashProgrammer(artifacts, "COM5")._image_arguments(
                files["board_data.bin"]
            )
            self.assertIn(hex(EXPECTED_CONFIG_OFFSET), arguments)
            self.assertIn(hex(EXPECTED_BOARD_DATA_OFFSET), arguments)
            self.assertIn(str(files["board_data.bin"]), arguments)
            self.assertNotIn("0x820000", arguments)
            self.assertNotIn("0xba0000", arguments)

    def test_config_fat_is_written_in_its_own_stage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            artifacts, files = self._artifacts(Path(directory))
            programmer = FlashProgrammer(artifacts, "COM5")

            with patch("manufacturing_tools.flash_programmer._run") as run:
                programmer.write_initial_images(files["board_data.bin"])
                programmer.format_config_storage()
                programmer.verify_and_run(files["board_data.bin"])

            commands = [call.args[0] for call in run.call_args_list]
            self.assertEqual(len(commands), 4)
            self.assertNotIn(str(files["config.bin"]), commands[0])
            self.assertIn(str(files["board_data.bin"]), commands[0])
            self.assertEqual(commands[1][-2], hex(EXPECTED_CONFIG_OFFSET))
            self.assertEqual(commands[1][-1], str(files["config.bin"]))
            self.assertIn("verify-flash", commands[2])
            self.assertIn(str(files["config.bin"]), commands[2])
            self.assertIn(str(files["board_data.bin"]), commands[2])
            self.assertEqual(commands[3][-1], "run")

    def test_config_image_preflight_rejects_erased_content(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "config.bin"
            image.write_bytes(b"\xff" * 1024)
            self.assertTrue(_image_is_fully_erased(image))

            image.write_bytes(b"\xff" * 511 + b"\x00" + b"\xff" * 512)
            self.assertFalse(_image_is_fully_erased(image))

    def test_manufacturing_generator_uses_512_byte_safe_mode(self) -> None:
        command = _config_generator_command(
            Path("python"), Path("idf"), Path("seed"), Path("config.bin")
        )
        self.assertIn("--sector_size", command)
        self.assertEqual(command[command.index("--sector_size") + 1], "512")
        self.assertIn("--wl_mode", command)
        self.assertEqual(command[command.index("--wl_mode") + 1], "safe")


if __name__ == "__main__":
    unittest.main()
