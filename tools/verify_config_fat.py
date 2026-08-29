"""Verify that a generated config FAT matches runtime Safety WL capacity."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import struct
import sys


FAT_SECTOR_SIZE = 512
WL_CONFIG_SECTORS = 1
WL_DUMMY_SECTORS = 1
WL_FLASH_SECTOR_SIZE = 4096
WL_SAFE_MODE_SECTORS = 2
WL_STATE_COPIES = 2
WL_STATE_HEADER_SIZE = 64
WL_STATE_RECORD_SIZE = 16

BPB_BYTES_PER_SECTOR_OFFSET = 11
BPB_TOTAL_SECTORS_16_OFFSET = 19
BPB_TOTAL_SECTORS_32_OFFSET = 32
BPB_SIGNATURE_OFFSET = 510
BPB_SIGNATURE = b"\x55\xaa"
WL_CONFIG_FORMAT = "<IIIIIIII"


class ConfigFatValidationError(ValueError):
    """The generated config FAT does not match the firmware storage contract."""


@dataclass(frozen=True)
class ConfigFatCapacity:
    """Capacity values extracted from and calculated for a config FAT image."""

    bytes_per_sector: int
    bpb_sectors: int
    runtime_sectors: int


def _round_up(value: int, alignment: int) -> int:
    return (value + alignment - 1) // alignment


def calculate_runtime_sector_count(partition_size: int) -> int:
    """Mirror ESP-IDF 6.0.2 Safety WL capacity exposed to FAT."""
    if partition_size <= 0 or partition_size % WL_FLASH_SECTOR_SIZE != 0:
        raise ConfigFatValidationError(
            "partition size must be a positive multiple of 4096 bytes"
        )

    physical_sectors = partition_size // WL_FLASH_SECTOR_SIZE
    state_bytes = (
        WL_STATE_HEADER_SIZE + physical_sectors * WL_STATE_RECORD_SIZE
    )
    state_sectors = _round_up(state_bytes, WL_FLASH_SECTOR_SIZE)
    runtime_physical_sectors = physical_sectors - (
        WL_STATE_COPIES * state_sectors
        + WL_CONFIG_SECTORS
        + WL_DUMMY_SECTORS
        + WL_SAFE_MODE_SECTORS
    )
    if runtime_physical_sectors <= 0:
        raise ConfigFatValidationError(
            "partition is too small for Safety WL metadata"
        )

    runtime_bytes = runtime_physical_sectors * WL_FLASH_SECTOR_SIZE
    if runtime_bytes % FAT_SECTOR_SIZE != 0:
        raise ConfigFatValidationError(
            "Safety WL capacity is not aligned to the FAT sector size"
        )
    return runtime_bytes // FAT_SECTOR_SIZE


def _read_u16(image: bytes, offset: int) -> int:
    return struct.unpack_from("<H", image, offset)[0]


def _read_u32(image: bytes, offset: int) -> int:
    return struct.unpack_from("<I", image, offset)[0]


def validate_config_fat_image(
    image: bytes, partition_size: int
) -> ConfigFatCapacity:
    """Validate image length, WL metadata, BPB, and runtime capacity."""
    if len(image) != partition_size:
        raise ConfigFatValidationError(
            "config FAT image size mismatch: "
            f"actual={len(image)} expected={partition_size}"
        )

    runtime_sectors = calculate_runtime_sector_count(partition_size)
    config_offset = partition_size - WL_FLASH_SECTOR_SIZE
    (
        start_address,
        full_memory_size,
        page_size,
        flash_sector_size,
        _update_rate,
        write_size,
        _version,
        _temporary_buffer_size,
    ) = struct.unpack_from(WL_CONFIG_FORMAT, image, config_offset)
    if start_address != 0:
        raise ConfigFatValidationError(
            f"WL start address mismatch: actual={start_address} expected=0"
        )
    if full_memory_size != partition_size:
        raise ConfigFatValidationError(
            "WL partition size mismatch: "
            f"actual={full_memory_size} expected={partition_size}"
        )
    if page_size != WL_FLASH_SECTOR_SIZE:
        raise ConfigFatValidationError(
            "WL page size mismatch: "
            f"actual={page_size} expected={WL_FLASH_SECTOR_SIZE}"
        )
    if flash_sector_size != WL_FLASH_SECTOR_SIZE:
        raise ConfigFatValidationError(
            "WL flash sector size mismatch: "
            f"actual={flash_sector_size} expected={WL_FLASH_SECTOR_SIZE}"
        )
    if write_size != WL_STATE_RECORD_SIZE:
        raise ConfigFatValidationError(
            "WL state record size mismatch: "
            f"actual={write_size} expected={WL_STATE_RECORD_SIZE}"
        )

    boot_offset = WL_FLASH_SECTOR_SIZE
    if image[
        boot_offset + BPB_SIGNATURE_OFFSET :
        boot_offset + BPB_SIGNATURE_OFFSET + len(BPB_SIGNATURE)
    ] != BPB_SIGNATURE:
        raise ConfigFatValidationError("FAT BPB signature is invalid")

    bytes_per_sector = _read_u16(
        image, boot_offset + BPB_BYTES_PER_SECTOR_OFFSET
    )
    if bytes_per_sector != FAT_SECTOR_SIZE:
        raise ConfigFatValidationError(
            "FAT sector size mismatch: "
            f"actual={bytes_per_sector} expected={FAT_SECTOR_SIZE}"
        )

    total_sectors_16 = _read_u16(
        image, boot_offset + BPB_TOTAL_SECTORS_16_OFFSET
    )
    total_sectors_32 = _read_u32(
        image, boot_offset + BPB_TOTAL_SECTORS_32_OFFSET
    )
    if (total_sectors_16 == 0) == (total_sectors_32 == 0):
        raise ConfigFatValidationError(
            "exactly one BPB total-sector field must be nonzero"
        )
    bpb_sectors = total_sectors_16 or total_sectors_32
    if bpb_sectors != runtime_sectors:
        raise ConfigFatValidationError(
            "FAT BPB and Safety WL runtime capacity mismatch: "
            f"actual={bpb_sectors} expected={runtime_sectors} sectors"
        )

    return ConfigFatCapacity(
        bytes_per_sector=bytes_per_sector,
        bpb_sectors=bpb_sectors,
        runtime_sectors=runtime_sectors,
    )


def _integer_argument(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Verify generated config FAT BPB against ESP-IDF Safety WL capacity"
        )
    )
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument(
        "--partition-size", required=True, type=_integer_argument
    )
    args = parser.parse_args()

    try:
        image = args.image.read_bytes()
        capacity = validate_config_fat_image(image, args.partition_size)
    except (OSError, ConfigFatValidationError) as error:
        print(f"config FAT validation failed: {error}", file=sys.stderr)
        return 1

    print(
        "config FAT verified: "
        f"sector_size={capacity.bytes_per_sector} "
        f"bpb_sectors={capacity.bpb_sectors} "
        f"runtime_sectors={capacity.runtime_sectors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
