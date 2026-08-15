from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
VERSION_HEADER = (ROOT / "SRC/firmware_version.h").read_text(encoding="utf-8")
ROOT_CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
UPDATE_SOURCE = (ROOT / "SRC/platform/firmware_update.c").read_text(
    encoding="utf-8"
)
WORKER_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")


class FirmwareVersionPolicyTests(unittest.TestCase):
    def test_manual_release_version_is_semantic_and_initially_0_1_0(self) -> None:
        match = re.search(
            r'^#define CBV_FIRMWARE_VERSION "([0-9]+\.[0-9]+\.[0-9]+)"$',
            VERSION_HEADER,
            re.MULTILINE,
        )
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), "0.1.0")

    def test_build_embeds_release_version_and_seven_digit_git_hash(self) -> None:
        self.assertIn('git rev-parse --short=7 HEAD', ROOT_CMAKE)
        self.assertIn(
            'set(PROJECT_VER "${CBV_RELEASE_VERSION}+${CBV_GIT_HASH}")',
            ROOT_CMAKE,
        )
        self.assertIn("CMAKE_CONFIGURE_DEPENDS", ROOT_CMAKE)

    def test_update_status_separates_version_and_hash(self) -> None:
        for state in ("WRITING", "STAGED", "CONFIRMED", "ROLLED_BACK"):
            with self.subTest(state=state):
                start = UPDATE_SOURCE.index(f'"state={state}\\r\\n')
                excerpt = UPDATE_SOURCE[start : start + 220]
                self.assertIn("version=", excerpt)
                self.assertIn("hash=", excerpt)
        self.assertIn('"version=-\\r\\nhash=-\\r\\n"', UPDATE_SOURCE)

    def test_board_and_diag_report_hash_without_removing_fingerprint(self) -> None:
        self.assertIn("firmware_hash=%s", WORKER_SOURCE)
        self.assertIn("version=%s hash=%s fingerprint=%s", WORKER_SOURCE)


if __name__ == "__main__":
    unittest.main()
