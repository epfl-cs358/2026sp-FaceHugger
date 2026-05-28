#pragma once
#include <stddef.h>
#include <stdint.h>
#include "../nervous_system/clips_all.h"

// Serializes clips[] to {"clips":[{"id":N,"name":"...","ms":N},...]}
// Returns bytes written into buf (null-terminated), or 0 on error.
size_t buildClipListJson(const FhClip* clips, uint8_t count, char* buf, size_t buflen);
