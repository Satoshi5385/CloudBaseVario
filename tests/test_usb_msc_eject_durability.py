import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MSC = (ROOT / "components/esp_tinyusb/tinyusb_msc.c").read_text(
    encoding="utf-8"
)
TINYUSB = (ROOT / "components/esp_tinyusb/tinyusb.c").read_text(
    encoding="utf-8"
)
SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")
LOCAL_PATCH = (ROOT / "components/esp_tinyusb/LOCAL_PATCH.md").read_text(
    encoding="utf-8"
)


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


class UsbMscEjectDurabilityTests(unittest.TestCase):
    def test_write_completion_queues_deferred_ownership_transition(self) -> None:
        write = function_body(
            MSC,
            "static void tusb_write_func(void *param)",
            "static inline esp_err_t msc_storage_write_sector_deferred",
        )
        self.assertIn("storage->deffered_writes--;", write)
        self.assertIn("tud_msc_async_io_done", write)
        self.assertIn("usbd_defer_func(tusb_apply_requested_mount", write)
        self.assertLess(
            write.index("tud_msc_async_io_done"),
            write.index("usbd_defer_func(tusb_apply_requested_mount"),
        )

    def test_mount_request_freezes_host_io_and_defers_while_pending(self) -> None:
        request = function_body(
            MSC,
            "static esp_err_t msc_storage_request_mount(\n"
            "    msc_storage_obj_t *storage, "
            "tinyusb_msc_mount_point_t mount_point)\n{",
            "static void tusb_apply_requested_mount",
        )
        self.assertIn("storage->host_io_enabled = false;", request)
        self.assertIn("pending_writes = storage->deffered_writes;", request)
        self.assertIn("return ESP_ERR_NOT_FINISHED;", request)
        self.assertLess(
            request.index("storage->host_io_enabled = false;"),
            request.index("pending_writes = storage->deffered_writes;"),
        )

    def test_sync_and_eject_use_the_same_drain_boundary(self) -> None:
        eject = function_body(
            MSC,
            "bool tud_msc_start_stop_cb",
            "int32_t tud_msc_read10_cb",
        )
        scsi = function_body(
            MSC,
            "int32_t tud_msc_scsi_cb",
            "/*********************************************************************** TinyUSB MSC callbacks*/",
        )
        self.assertIn("msc_storage_request_mount", eject)
        self.assertIn("return false;", eject)
        self.assertIn("msc_storage_sync_lun(lun)", scsi)
        self.assertIn("ret = -1;", scsi)

    def test_usb_unmount_uses_drain_aware_mount_request(self) -> None:
        unmount = function_body(
            TINYUSB,
            "void tud_umount_cb",
            "#ifdef CONFIG_TINYUSB_SUSPEND_CALLBACK",
        )
        mount_all = function_body(
            MSC,
            "void msc_storage_mount_to_app(void)",
            "void msc_storage_mount_to_usb(void)",
        )
        self.assertIn("msc_storage_mount_to_app();", unmount)
        self.assertIn("msc_storage_request_mount", mount_all)
        self.assertIn("ESP_ERR_NOT_FINISHED", mount_all)

    def test_spec_requires_drain_before_app_mount(self) -> None:
        self.assertIn("受理済みWRITEが残る場合はAPP側mountを遅延", SPEC)
        self.assertIn("非同期完了処理が終わった後にだけ所有権をAPP側へ戻す", SPEC)
        self.assertIn("defer application mounting", LOCAL_PATCH)


if __name__ == "__main__":
    unittest.main()
