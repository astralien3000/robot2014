#ifndef FPGA_HPP
#define FPGA_HPP

#include <base/integer.hpp>

///////////////////////////////////////////////
// Functions

void fpga_init(void);

///////////////////////////////////////////////
// On UNIOC
#if defined (__AVR_ATmega128__)

#define ENC_R      (*(volatile u32*)0x80A0)
#define ENC_L      (*(volatile u32*)0x8098)
#define ENC_MOT_R  (*(volatile u32*)0x809C)
#define ENC_MOT_L  (*(volatile u32*)0x8094)

#define MOT_R      (*(volatile s8*)0x8000)
#define MOT_L      (*(volatile s8*)0x8001)
#define RESET_FPGA (*(volatile u8*)0x807F)

#define RELATION   (*(volatile u32*)0x8024)

// READ-ONLY
#define FPGA_US    (*(volatile u16*)0x8080)
#define FPGA_MS    (*(volatile u16*)0x8082)
#define FPGA_S     (*(volatile u16*)0x8084)

#define POSX_FPGA  (*(volatile s32*)0x8088)
#define POSY_FPGA  (*(volatile s32*)0x808C)
#define ROT_FPGA   (*(volatile u16*)0x8090)

///////////////////////////////////////////////
// NO UNIOC
#else

extern volatile u32 ENC_R;
extern volatile u32 ENC_L;
extern volatile u32 ENC_MOT_R;
extern volatile u32 ENC_MOT_L;

extern volatile s8 MOT_R;
extern volatile s8 MOT_L;
extern volatile u8 RESET_FPGA;

extern volatile u32 RELATION;

// READ-ONLY
extern volatile u16 FPGA_US;
extern volatile u16 FPGA_MS;
extern volatile u16 FPGA_S;
extern volatile u16 POSX_FPGA;
extern volatile u16 POSY_FPGA;
extern volatile u16 ROT_FPGA;

#endif//UNIOC

#endif//FPGA_HPP
