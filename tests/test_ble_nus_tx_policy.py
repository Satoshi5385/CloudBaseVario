from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
DOMAIN = (ROOT / "SRC/domain/ble_nus_tx.c").read_text(encoding="utf-8")
DOMAIN_HEADER = (ROOT / "SRC/domain/ble_nus_tx.h").read_text(encoding="utf-8")
PLATFORM = (ROOT / "SRC/platform/ble_vario.c").read_text(encoding="utf-8")
PLATFORM_HEADER = (ROOT / "SRC/platform/ble_vario.h").read_text(encoding="utf-8")
WORKER = (ROOT / "SRC/app/ble_tx_worker.c").read_text(encoding="utf-8")
DIAGNOSTICS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
CI = "\n".join(
    (
        (ROOT / ".github/workflows/build.yml").read_text(encoding="utf-8"),
        (ROOT / "ci/run_ci.sh").read_text(encoding="utf-8"),
    )
)
SDKCONFIG_DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")


class BleNusTxPolicyTests(unittest.TestCase):
    def test_platform_only_submits_one_generation_checked_fragment(self) -> None:
        self.assertIn("ble_vario_notify_nus_fragment", PLATFORM_HEADER)
        self.assertIn("generation == nus_link_generation", PLATFORM)
        self.assertIn("length > payload_capacity", PLATFORM)
        self.assertNotIn("ble_vario_notify_lk8ex1", PLATFORM_HEADER)
        self.assertNotIn("ble_vario_notify_gps_pair", PLATFORM_HEADER)
        self.assertNotIn("BLE_GAP_EVENT_NOTIFY_TX", PLATFORM)

    def test_scheduler_owns_latest_only_and_round_robin_policy(self) -> None:
        self.assertIn("pending_lk8ex1", DOMAIN_HEADER)
        self.assertIn("pending_gps", DOMAIN_HEADER)
        self.assertIn("last_selected_source", DOMAIN_HEADER)
        self.assertIn("BLE_NUS_TX_OFFER_REPLACED", DOMAIN)
        self.assertIn("state->last_selected_source == BLE_NUS_TX_SOURCE_LK8EX1", DOMAIN)
        self.assertIn("state->resync_pending = true", DOMAIN)

    def test_worker_enforces_four_token_non_accumulating_budget(self) -> None:
        self.assertIn("BLE_NUS_TX_TOKENS_PER_INTERVAL 4U", DOMAIN_HEADER)
        self.assertIn(
            "state->available_tokens = BLE_NUS_TX_TOKENS_PER_INTERVAL;",
            DOMAIN,
        )
        self.assertIn("ble_nus_tx_take_token(tx_state)", WORKER)
        self.assertIn("ble_nus_tx_defer_until_refill(tx_state)", WORKER)
        self.assertIn("DEFAULT_CONNECTION_INTERVAL_US UINT32_C(30000)", WORKER)
        self.assertIn("BLE_CONNECTION_INTERVAL_MIN_MS UINT32_C(15)", PLATFORM)
        self.assertIn("BLE_CONNECTION_INTERVAL_MAX_MS UINT32_C(30)", PLATFORM)

    def test_diagnostics_and_ci_cover_the_new_contract(self) -> None:
        for field in (
            "fragment_attempts=",
            "fragment_accepted=",
            "fragment_errors=",
            "lk8_coalesced=",
            "gps_coalesced=",
            "partial_aborts=",
            "stream_resyncs=",
        ):
            self.assertIn(field, DIAGNOSTICS)
        self.assertIn("tests/test_ble_nus_tx.c", CI)
        self.assertIn("CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU 247", CI)
        self.assertIn("CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=247", SDKCONFIG_DEFAULTS)
        self.assertIn("CONFIG_BT_NIMBLE_MAX_CCCDS 3", CI)


if __name__ == "__main__":
    unittest.main()
