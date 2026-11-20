#ifndef INSPIRECV_STREAMTASK_CORE_ST_DEFS_H_
#define INSPIRECV_STREAMTASK_CORE_ST_DEFS_H_

#include <stdio.h>
#include <assert.h>

// External config header removed; macros are provided via compile definitions.

// Visibility
#ifndef INSPIRECV_TASK_PUBLIC
#define INSPIRECV_TASK_PUBLIC
#endif

// Logging / assert
#ifndef INSPIRECV_TASK_PRINT
#define INSPIRECV_TASK_PRINT(format, ...) printf(format, ##__VA_ARGS__)
#endif
#ifndef INSPIRECV_TASK_ERROR
#define INSPIRECV_TASK_ERROR(format, ...) printf(format, ##__VA_ARGS__)
#endif
#ifndef INSPIRECV_TASK_ASSERT
#define INSPIRECV_TASK_ASSERT(x) assert(x)
#endif

#endif // INSPIRECV_STREAMTASK_CORE_ST_DEFS_H_



