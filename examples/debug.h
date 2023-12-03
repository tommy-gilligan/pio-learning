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
    int sm0_clkdiv;
    int sm0_execctrl;
    int sm0_shiftctrl;
    int sm0_addr;
    int sm0_instr;
    int sm0_pinctrl;
};

void print_pio_state() {
    char buff[300];
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
    p = json_int( p, "sm0_clkdiv", *((volatile int *) 0x502000c8), &len );
    p = json_int( p, "sm0_execctrl", *((volatile int *) 0x502000cc), &len );
    p = json_int( p, "sm0_shiftctrl", *((volatile int *) 0x502000d0), &len );
    p = json_int( p, "sm0_addr", *((volatile int *) 0x502000d4), &len );
    p = json_int( p, "sm0_instr", *((volatile int *) 0x502000d8), &len );
    p = json_int( p, "sm0_pinctrl", *((volatile int *) 0x502000dc), &len );
    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}
