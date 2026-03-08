#ifndef NAV_FLASH_H_
#define NAV_FLASH_H_

#include "zf_common_headfile.h"
#include "Inertial_Navigation.h"

void flash_save_nag(void);
uint8_t flash_load_nag(void);

#endif
