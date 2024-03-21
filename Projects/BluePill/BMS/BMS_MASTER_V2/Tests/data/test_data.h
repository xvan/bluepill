#ifndef TEST_DATA_H
#define TEST_DATA_H

#include "thomas.h"

#define TEST_DATA_COLUMNS 2
#define TEST_DATA_ROWS (sizeof(bin2c_thomas_bin)/sizeof(float)/TEST_DATA_COLUMNS)

static const float (* test_data)[TEST_DATA_COLUMNS] = (const float (*)[TEST_DATA_COLUMNS]) bin2c_thomas_bin;

#endif // TEST_DATA_H
