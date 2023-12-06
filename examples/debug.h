#include "json-maker/json-maker.h"
#include <stdio.h>
#include <limits.h>

struct pio_state_copy {
    unsigned int ctrl;
    unsigned int fstat;
    unsigned int fdebug;
    unsigned int flevel;
    unsigned int irq;
    unsigned int dbg_padout;
    unsigned int dbg_padoe;
    unsigned int dbg_cfginfo;
};

struct sm_state_copy {
    unsigned int sm_clkdiv;
    unsigned int sm_execctrl;
    unsigned int sm_shiftctrl;
    unsigned int sm_addr;
    unsigned int sm_instr;
    unsigned int sm_pinctrl;
};

#define SM0_BASE 0x502000c8
#define SM1_BASE 0x502000e0
#define SM2_BASE 0x502000f8
#define SM3_BASE 0x50200110

typedef struct {
	char v[33];
} UNSIGNED_INT_BIT_PATTERN;

char* format_byte(char* dest, char const* name, int address, size_t* len) {
    unsigned int byte = *((volatile unsigned int *) address);

    UNSIGNED_INT_BIT_PATTERN result = { 0 };

    int i = 32;

    while (i--) {
    	result.v[i] = '0' + (((byte << i) >> 31) & 1);
    }

    return json_nstr(dest, name, result.v, 33, len);
}

void print_sm_state(int base) {
    char buff[2000];
    size_t len = sizeof(buff);

    char* p = buff;
    p = json_objOpen( p, NULL, &len );

    p = format_byte(p, "sm_clkdiv", base, &len);
    p = format_byte(p, "sm_execctrl", base + 4, &len);
    p = format_byte(p, "sm_shiftctrl", base + 8, &len);
    p = format_byte(p, "sm_addr", base + 12, &len);
    p = format_byte(p, "sm_instr", base + 16, &len);
    p = format_byte(p, "sm_pinctrl", base + 20, &len);

    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}

void print_pio_state() {
    char buff[2000];
    size_t len = sizeof(buff);

    char* p = buff;
    p = json_objOpen( p, NULL, &len );
    p = format_byte(p, "ctrl", 0x50200000, &len);
    p = format_byte(p, "fstat", 0x50200004, &len);
    p = format_byte(p, "fdebug", 0x50200008, &len);
    p = format_byte(p, "flevel", 0x5020000c, &len);
    p = format_byte(p, "irq", 0x50200030, &len);
    p = format_byte(p, "dbg_padout", 0x5020003c, &len);
    p = format_byte(p, "dbg_padoe", 0x50200040, &len);
    p = format_byte(p, "dbg_cfginfo", 0x50200044, &len);

    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}
