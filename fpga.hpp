#ifndef FPGA_HPP
#define FPGA_HPP

///////////////////////////////////////////////
// On UNIOC
#if defined (__AVR_ATmega128__)

#define ENC_R (*(volatile u32*)0x80A0)
#define ENC_L (*(volatile u32*)0x8098)
#define ENC_MOT_R (*(volatile u32*)0x8094)
#define ENC_MOT_L (*(volatile u32*)0x809C)

#define MOT_R (*(volatile s8*)0x8000)
#define MOT_L (*(volatile s8*)0x8001)
#define RESET_FPGA (*(volatile u8*)0x807F)

#define RELATION (*(volatile u32*)0x8024)

// READ-ONLY
#define FPGA_US   (*(volatile u16*)0x8080)
#define FPGA_MS   (*(volatile u16*)0x8082)
#define FPGA_S    (*(volatile u16*)0x8084)
#define POSX_FPGA    (*(volatile u32*)0x8086)
#define POSY_FPGA    (*(volatile u32*)0x808A)
#define ROT_FPGA     (*(volatile u16*)0x808E)

///////////////////////////////////////////////
//NO UNIOC
#else

#warning "FPGA is not managed"

u32 ENC_R;
u32 ENC_L;
u32 ENC_MOT_R;
u32 ENC_MOT_L;

s8 MOT_R;
s8 MOT_L;
u8 RESET_FPGA;

u32 RELATION;

// READ-ONLY
u16 FPGA_US;
u16 FPGA_MS;
u16 FPGA_S;
u16 POSX_FPGA;
u16 POSY_FPGA;
u16 ROT_FPGA;

#endif//UNIOC

#endif//FPGA_HPP
