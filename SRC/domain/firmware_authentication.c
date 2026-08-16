#include "domain/firmware_authentication.h"

const char *firmware_authenticity_name(firmware_authenticity_t authenticity) {
    switch (authenticity) {
    case FIRMWARE_AUTH_OFFICIAL:
        return "OFFICIAL";
    case FIRMWARE_AUTH_NON_OFFICIAL:
        return "NON_OFFICIAL";
    case FIRMWARE_AUTH_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}
