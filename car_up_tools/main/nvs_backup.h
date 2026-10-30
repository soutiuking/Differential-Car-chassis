#pragma once

#include <stdbool.h>

void backup_init(void);
bool backup_save(void);
bool backup_restore(void);
bool backup_has(void);
