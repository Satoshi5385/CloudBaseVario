from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
USB = (ROOT / "SRC/platform/usb_device_service.c").read_text(encoding="utf-8")
UPDATE = (ROOT / "SRC/platform/firmware_update.c").read_text(encoding="utf-8")
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
MSC = (ROOT / "components/esp_tinyusb/tinyusb_msc.c").read_text(
    encoding="utf-8"
)
PSRAM = (ROOT / "components/esp_tinyusb/storage_psram.c").read_text(
    encoding="utf-8"
)
BUFFER = (ROOT / "components/esp_tinyusb/storage_psram_buffer.c").read_text(
    encoding="utf-8"
)
HEADER = (ROOT / "components/esp_tinyusb/include/tinyusb_msc.h").read_text(
    encoding="utf-8"
)
DIAG = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
PENDING = (ROOT / "SRC/domain/firmware_pending_record.h").read_text(
    encoding="utf-8"
)


def body(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    return source[begin : source.index(end, begin)]


class PsramRecoveryMscTests(unittest.TestCase):
    def test_fixed_geometry_and_psram_capability_contract(self) -> None:
        self.assertIn("RECOVERY_DISK_BYTES UINT32_C(4194304)", USB)
        self.assertIn("RECOVERY_SECTOR_BYTES UINT32_C(512)", USB)
        self.assertIn('RECOVERY_VOLUME_LABEL "CBVUPDATE"', USB)
        self.assertIn('RECOVERY_MOUNT_PATH "/update"', USB)
        self.assertIn("esp_psram_is_initialized()", USB)
        self.assertIn("heap_caps_get_largest_free_block", USB)
        self.assertIn("MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT", USB)
        self.assertIn("heap_caps_calloc(", USB)

    def test_psram_medium_checks_every_byte_range(self) -> None:
        for operation in ("SIZE_MAX /", "SIZE_MAX -", "end <= buffer->size_bytes"):
            self.assertIn(operation, BUFFER)
        self.assertIn("storage_psram_buffer_read", PSRAM)
        self.assertIn("storage_psram_buffer_write", PSRAM)
        self.assertIn("ff_diskio_register", PSRAM)
        self.assertLess(
            PSRAM.index("command == CTRL_SYNC"),
            PSRAM.index("buffer == NULL", PSRAM.index("command == CTRL_SYNC")),
        )
        self.assertIn("msc_storage_range_valid", MSC)
        self.assertIn("tinyusb_msc_new_storage_psram", HEADER)
        self.assertIn("storage_caps |= MALLOC_CAP_INTERNAL", MSC)

    def test_format_targets_the_allocated_psram_drive(self) -> None:
        self.assertIn(
            "static esp_err_t vfs_fat_format(const char *drv,",
            MSC,
        )
        self.assertIn("f_mkfs(drv,", MSC)
        self.assertNotIn('f_mkfs(""', MSC)
        self.assertIn("vfs_fat_format(drv, format_flags)", MSC)
        self.assertIn("vfs_fat_mount(drv, fs, true)", MSC)

    def test_recovery_does_not_fallback_to_flash_lun(self) -> None:
        recovery = body(
            USB,
            "esp_err_t usb_device_recovery_storage_init(",
            "esp_err_t usb_device_recovery_storage_deinit(",
        )
        self.assertIn("tinyusb_msc_new_storage_psram", recovery)
        self.assertNotIn("create_msc_storage", recovery)
        self.assertNotIn("tinyusb_msc_new_storage_spiflash", recovery)
        self.assertIn("goto fail;", recovery)
        self.assertIn("write_recovery_init_failure_status(ret)", recovery)

    def test_vbus_loss_stops_io_then_zeroes_and_frees_disk(self) -> None:
        stop = body(STARTUP, "static void recovery_stop(", "static void run_recovery_mode(")
        deinit = body(
            USB,
            "esp_err_t usb_device_recovery_storage_deinit(",
            "bool usb_device_recovery_storage_ready(",
        )
        self.assertLess(stop.index("usb_device_stop();"), stop.index("usb_device_recovery_storage_deinit();"))
        self.assertIn("tinyusb_msc_delete_storage", deinit)
        self.assertIn("mbedtls_platform_zeroize(recovery_psram_buffer", deinit)
        self.assertIn("heap_caps_free(recovery_psram_buffer)", deinit)

    def test_psram_ota_rechecks_digest_and_sets_boot_last(self) -> None:
        apply = body(
            UPDATE,
            "static esp_err_t apply_update(",
            "bool firmware_update_running_image_pending_verify(",
        )
        self.assertIn("UPDATE_IO_BUFFER_BYTES 8192U", UPDATE)
        self.assertIn("psa_hash_update", apply)
        self.assertIn("info->authentication.payload_sha256", apply)
        self.assertIn("usb_device_vbus_present()", apply)
        self.assertIn("esp_ota_abort", apply)
        self.assertIn("ret = write_status(", apply)
        self.assertLess(apply.index("write_pending_record(target, info)"), apply.index("esp_ota_set_boot_partition(target)"))
        self.assertLess(apply.index("write_status("), apply.rindex("esp_ota_set_boot_partition(target)"))

    def test_pending_record_and_legacy_container_are_both_reconciled(self) -> None:
        reconcile = body(
            UPDATE,
            "static esp_err_t reconcile_pending_file(",
            "static esp_err_t apply_update(",
        )
        for field in (
            "format_version",
            "record_size",
            "payload_size",
            "target_address",
            "app_elf_sha256",
            "version",
            "crc32",
        ):
            self.assertIn(field, PENDING)
        self.assertIn("read_pending_record", reconcile)
        self.assertIn("inspect_image_at", reconcile)
        self.assertIn("esp_ota_get_last_invalid_partition", reconcile)
        self.assertIn("pending record invalid", reconcile)

        confirmation = body(
            UPDATE,
            "static void confirmation_task(",
            "esp_err_t firmware_update_begin_confirmation(",
        )
        self.assertLess(
            confirmation.index("pending_record_matches"),
            confirmation.index("esp_ota_mark_app_valid_cancel_rollback"),
        )

    def test_diagnostics_include_medium_allocation_io_source_and_digest(self) -> None:
        for field in (
            "recovery_medium=",
            "psram_largest_before=",
            "psram_alloc_error=",
            "psram_read_errors=",
            "psram_write_errors=",
            "source=%s digest_verified=%d",
        ):
            self.assertIn(field, DIAG)
        self.assertIn('strcmp(command, "DIAG STATUS")', STARTUP)
        self.assertIn("recovery_write_diagnostics();", STARTUP)


if __name__ == "__main__":
    unittest.main()
