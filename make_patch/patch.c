#include <stdint.h>

int lodivider_get_sub_band_range(int param_1) {
  if (param_1 == 10) {
    return 5;
  }
  if (param_1 < 0xb) {
    if (param_1 == 4) {
      return 1;
    }
    if (param_1 == 5) {
      return 4;
    }
    if (param_1 == 6) {
      return 2;
    }
  }
  else {
    if (param_1 == 0xc) {
      return 3;
    }
    if (param_1 == 0xf) {
      return 6;
    }
    if (param_1 == 0x1e) {
      return 7;
    }
    if (param_1 == 0x16) {
      return 7;
    }
  }
  return 0xffffffff;
}


__attribute__((section(".PROP_PATCH_VEC"))) uint32_t _PROP_PATCH_VEC[] = {
    (uint32_t) lodivider_get_sub_band_range
};
