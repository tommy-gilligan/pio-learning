#include "json-maker/json-maker.h"

struct pio_state_copy {
    int ctrl;
    int fstat;
    int fdebug;
    int flevel;
    int irq;
    int dbg_padout;
    int dbg_padoe;
    int dbg_cfginfo;
};

struct sm_state_copy {
    int sm_clkdiv;
    int sm_execctrl;
    int sm_shiftctrl;
    int sm_addr;
    int sm_instr;
    int sm_pinctrl;
};

#define SM0_BASE 0x502000c8
#define SM1_BASE 0x502000e0
#define SM2_BASE 0x502000f8
#define SM3_BASE 0x50200110

void print_sm_state(int base) {
    char buff[200];
    size_t len = sizeof(buff);

    char* p = buff;
    p = json_objOpen( p, NULL, &len );

    p = json_int( p, "sm_clkdiv", *((volatile int *) base), &len );
    p = json_int( p, "sm_execctrl", *((volatile int *) (base + 4)), &len );
    p = json_int( p, "sm_shiftctrl", *((volatile int *) (base + 8)), &len );
    p = json_int( p, "sm_addr", *((volatile int *) (base + 12)), &len );
    p = json_int( p, "sm_instr", *((volatile int *) (base + 16)), &len );
    p = json_int( p, "sm_pinctrl", *((volatile int *) (base + 20)), &len );

    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}

void print_pio_state() {
    char buff[200];
    size_t len = sizeof(buff);

    char* p = buff;
    p = json_objOpen( p, NULL, &len );
    p = json_int( p, "ctrl", *((volatile int *) 0x50200000), &len );
    p = json_int( p, "fstat", *((volatile int *) 0x50200004), &len );
    p = json_int( p, "fdebug", *((volatile int *) 0x50200008), &len );
    p = json_int( p, "flevel", *((volatile int *) 0x5020000c), &len );
    p = json_int( p, "irq", *((volatile int *) 0x50200030), &len );
    p = json_int( p, "dbg_padout", *((volatile int *) 0x5020003c), &len );
    p = json_int( p, "dbg_padoe", *((volatile int *) 0x50200040), &len );
    p = json_int( p, "dbg_cfginfo", *((volatile int *) 0x50200044), &len );

    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}
