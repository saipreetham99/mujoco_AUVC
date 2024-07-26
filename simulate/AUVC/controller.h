#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mujoco/mujoco.h>

typedef void (*plug_test_t)(void);
typedef int (*plug_init_t)(mjModel *m, mjData *d);
typedef int (*plug_update_t)(mjModel *m, mjData *d);

extern "C" void test_func(void);
extern "C" int initPlug(mjModel *m, mjData *d);
extern "C" bool updatePlug(mjModel *m, mjData *d);

#endif // CONTROLLER_H
