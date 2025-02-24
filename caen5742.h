/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * caen5742.h
 * Caen digitizer low level driver */

#ifndef __CAEN5742_H
#define __CAEN5742_H

#include "lib/CAENComm.h"
#include "lib/CAENDigitizerType.h"

//------------------------------------------------------------------------------
//DEVICE defines
//CAEN DT5742 16+1 Channel 12 bit 5 GS/s Switched Capacitor Digitizer

//channel geometry
#define CAEN_DT7542_ADCRES 0xFFF   //ADC resolution 12b
#define CAEN_DT7542_SIGWIN 1024    //ADC digitizing window
#define CAEN_DT7542_CH_NUM 8       //channel number per group
#define CAEN_DT7542_GR_NUM 2       //groups number = fast trigger digitized ch number (1 per DRS4)
#define CAEN_DT7542_CC_NUM CAEN_DT7542_CH_NUM*CAEN_DT7542_GR_NUM //18 channels no trigger
#define CAEN_DT7542_CT_NUM CAEN_DT7542_CC_NUM+CAEN_DT7542_GR_NUM //18 channels total number
#define CAEN_DT7542_HDR_SZ 4       //caen header size, longwords (LW)

#define SVAL_MAX_LEN 16     //max string length of register value
#define SRES_MAX_LEN 48     //max string length of resultat string

//this driver specific error codes
typedef enum caen_drv_err_t {
    drv_err_ok          =     0, //ok
    drv_err_not_found   =  -100, //reg name not found
    drv_err_ro          =  -101, //read only reg
    drv_err_wo          =  -102, //write only regб cannot read broadcast
    drv_err_spi_busy    =  -103, //SPI is busy, cannot prop read reg
    drv_err_blt_hdr_sz  =  -104, //BTL header size mismatch
    drv_err_blt_hdr_cor =  -105, //BTL header footprint 0101 corrupted
    drv_err_blt_zero    =  -106, //nothing read, hdr payload=0
    drv_err_mem_outr    =  -107, //mem buf out of range
} caen_drv_err;

//reg masks
#define UINT01_MAX 0x0001   // only first bit value mask
#define UINT02_MAX 0x0003   // 2bit value mask
#define UINT04_MAX 0x000F   // 4bit value mask
#define UINT08_MAX 0x00FF   // 8bit value mask
#define UINT10_MAX 0x07FF   //10bit value mask
#define UINT12_MAX 0x0FFF   //12bit value mask
//#define UINT16_MAX 0xFFFF //16bit value mask
#define UINT24_MAX 0xFFFFFF //24bit value mask
//#define UINT32_MAX 0xFFFFFFFF //32bit value mask
#define STRING_MAX 0x0       //string stub

//define X742 additional registers
//"BASE" for base 0x10xx channel number access, "ADD" for broadcast addr 0x80xx
#define CAEN_DGTZ_REG_RO                       (0xFFFF-0) //read  only register stub
#define CAEN_DGTZ_REG_WO                       (0xFFFF-1) //write only register stub
#define CAEN_DGTZ_SW_REL_FAKE_ADDRESS          (0xFFFF-2) //Fake addr for SW rel read

#define CAEN_DGTZ_DUMMY_BASE_ADDRESS               0x1024 //Dummy BASE
#define CAEN_DGTZ_DUMMY_ADD                        0x8024 //Dummy ALL
#define CAEN_DGTZ_POST_TRIG_BASE_ADDRESS           0x1014 //Post Trigger
#define CAEN_DGTZ_X742_DRS4_TEMP_BASE_ADDRESS      0x10A0 //DRS4 Chip Temperature

#define CAEN_DGTZ_CHANNEL_SELECTION_ADD            0x10A4 //Group n Channel Selection for 1098 and 8080
#define CAEN_DGTZ_CHANNEL_THRESHOLD_ADD            0x8080 //Group n Channel Threshold ALL
#define CAEN_DGTZ_CHANNEL_DAC_ADD                  0x8098 //Group n Channel DC Offset ALL
#define CAEN_DGTZ_CHANNEL_GROUP_V1740_ADD          0x80A8 //Group n Channel Trigger Mask ALL
#define CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_ADD      0x80D4 //Group n TR Trigger Threshold ALL
#define CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_ADD 0x80DC //Group n TR DC Offset ALL

//------------------------------------------------------------------------------
//caen digitizer registers address structure and data mask

//string name, register, mask
typedef struct caen_reg_addr_t{
  const char* name; //command or setting name as a text
  uint16_t addrR;   //register address for reading or =0 if nonR
  uint16_t addrW;   //register address for writing or =0 if nonW
  uint32_t mask;    //mask for data read or write
} caen_reg_addr;

//caen digitizer named registers map
const caen_reg_addr caen_reg_map[] = {
    //0x1n24, 0x8024 Dummy32: [31.0]
    {"dummy",       CAEN_DGTZ_DUMMY_BASE_ADDRESS, CAEN_DGTZ_DUMMY_ADD, UINT32_MAX},
    
    //0x10D8 rd, 0x80D8 wr - DRS4 Sampling Frequency
    {"drs4_freq",   CAEN_DGTZ_DRS4_FREQUENCY_REG, CAEN_DGTZ_DRS4_FREQUENCY_REG_WRITE, UINT02_MAX},//00-10
    {"drs4_temp",   CAEN_DGTZ_X742_DRS4_TEMP_BASE_ADDRESS, CAEN_DGTZ_REG_RO, UINT08_MAX},
    {"rec_len",     CAEN_DGTZ_CUSTOM_SIZE_REG, CAEN_DGTZ_CUSTOM_SIZE_REG, UINT02_MAX}, //00-10
    {"io_lev",      CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, UINT01_MAX},
    
    //versions
    {"ver_amc",     CAEN_DGTZ_CHANNEL_AMC_FPGA_FW_BASE_ADDRESS, CAEN_DGTZ_REG_RO, UINT32_MAX},
    {"ver_roc",     CAEN_DGTZ_FW_REV_ADD,                       CAEN_DGTZ_REG_RO, UINT32_MAX},
    {"ver_mem",     CAEN_DGTZ_BOARD_INFO_ADD,                   CAEN_DGTZ_REG_RO, UINT24_MAX},
    {"ver_drv",     CAEN_DGTZ_SW_REL_FAKE_ADDRESS,              CAEN_DGTZ_REG_RO, STRING_MAX},
    
    {"sel_gr",      CAEN_DGTZ_CH_ENABLE_ADD, CAEN_DGTZ_CH_ENABLE_ADD, UINT04_MAX},
    
    //0x8000 board config
    {"tr_polar",       CAEN_DGTZ_BROAD_CH_CTRL_ADD, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, (1UL<<06)},
    {"tr_fast_dgtize", CAEN_DGTZ_BROAD_CH_CTRL_ADD, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, (1UL<<11)},
    
    {"mode_test",      CAEN_DGTZ_BROAD_CH_CTRL_ADD, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, (1UL<<3) },
    {"force_gpo",      CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, (1UL<<14)},
    {"sw_reset",       CAEN_DGTZ_REG_WO, CAEN_DGTZ_SW_RESET_ADD, UINT32_MAX},

    {"evt_blt",        CAEN_DGTZ_BLT_EVENT_NUM_ADD, CAEN_DGTZ_BLT_EVENT_NUM_ADD,       UINT10_MAX},
    {"tr_delay",       CAEN_DGTZ_POST_TRIG_BASE_ADDRESS, CAEN_DGTZ_POST_TRIG_ADD,      UINT10_MAX},

    //ACQ
    {"acq_run",       CAEN_DGTZ_ACQ_CONTROL_ADD, CAEN_DGTZ_ACQ_CONTROL_ADD, (1UL<<2)},
    {"tr_soft_kick",  CAEN_DGTZ_REG_WO, CAEN_DGTZ_SW_TRIGGER_ADD, UINT32_MAX},

    //fast 0x8000.12, slow 0x810C.30, soft 0x810C.31, self 0x1nA8/0x80A8 at least one ch
    {"tr_fast",        CAEN_DGTZ_BROAD_CH_CTRL_ADD, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD,  (1UL<<12)},
    {"tr_slow",        CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD, CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD, (1UL<<30)},
    {"tr_soft",        CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD, CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD, (1UL<<31)},
    {"tr_self",        CAEN_DGTZ_REG_WO, CAEN_DGTZ_CHANNEL_GROUP_V1740_ADD, UINT08_MAX}, //2DO

    //0x1n98, 0x8098 - Group n Channel DC Offset: [19.16]=ch(4b), [15.0]=val(16b) 
    //set 0x1nA4 [3.0]=ch(4b) before read, check 0x1n88 bit[2]=0 before write
    {"gr_ch_dc",       CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS, CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS, UINT16_MAX},
    {"gr_ch_dc_all",   CAEN_DGTZ_CHANNEL_DAC_ADD,          CAEN_DGTZ_CHANNEL_DAC_ADD,          UINT16_MAX},

    //0x1n80, 0x8080 - Group n Channel Self-Tr Threshold: [15,12]=ch(4b), [11.0]=val(12b)
    //set 0x1nA4 [3.0]=ch(4b) before read, check 0x1n88 bit[2]=0 before write
    {"gr_ch_str_tr",      CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS, CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS, UINT12_MAX},
    {"gr_ch_str_tr_all",  CAEN_DGTZ_CHANNEL_THRESHOLD_ADD,          CAEN_DGTZ_CHANNEL_THRESHOLD_ADD,          UINT12_MAX},

    //0x1nA8, 0x80A8 - Group n Channel Self-Trigger Mask: [7.0] ch
    {"gr_ch_str_msk",      CAEN_DGTZ_CHANNEL_GROUP_V1740_BASE_ADDRESS, CAEN_DGTZ_CHANNEL_GROUP_V1740_BASE_ADDRESS, UINT08_MAX},
    {"gr_ch_str_msk_all",  CAEN_DGTZ_REG_WO,                           CAEN_DGTZ_CHANNEL_GROUP_V1740_ADD,          UINT08_MAX},

    //0x1nD4, 0x80D4 - Group n Fast-TR Trigger Threshold: [15.0]=thr_val(16b)
    //check 0x1n88 bit[2]=0 before write
    {"gr_ftr_tr",       CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_BASE_ADDRESS, CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_BASE_ADDRESS, UINT16_MAX},
    {"gr_ftr_tr_all",   CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_ADD,          CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_ADD,          UINT16_MAX},

    //0x1nDC, 0x80DC - Group n Fast-TR DC Offset: [15.0]=offs_val(16)
    //check 0x1n88 bit[2]=0 before write
    {"gr_ftr_dc",       CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS, CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS, UINT16_MAX},
    {"gr_ftr_dc_off",   CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_ADD,         CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_ADD,           UINT16_MAX},
  
};

//------------------------------------------------------------------------------
//DEBUG-REAL switch

//#define TARGET_DEVICE_CAENCOMM //real device via usb or connet
#define TARGET_DEVICE_STUB //stupid software stub device for test

//#define PRN_DIAG  printf //verbose print diagnostic
#define PRN_DIAG         //no print diagnostic

 //acquisiton status
 typedef struct acq_stat_t {
    unsigned char acq_run   : 1;
    unsigned char evt_rdy   : 1;
    unsigned char evt_full  : 1;
    unsigned char pll_unlck : 1;
    unsigned char board_rdy : 1;
    unsigned char trg_in    : 1;
 } acq_stat;

//Define real device functions
#ifdef  TARGET_DEVICE_CAENCOMM
#define DEVICE_OPEN(LinkType, LinkNum, ConetNode, VMEBaseAddress, handle) CAENComm_OpenDevice (LinkType, LinkNum, ConetNode, VMEBaseAddress, handle)
#define DEVICE_CLOSE(handle) CAENComm_CloseDevice(handle)
#define DEVICE_SW_VER(SwRel) CAENComm_SWRelease(SwRel)
#define DEVICE_REG_READ32(handle, Address, Data)  CAENComm_Read32 (handle, Address, Data)
#define DEVICE_REG_WRITE32(handle, Address, Data) CAENComm_Write32(handle, Address, Data)
#define DEVICE_ACQ_STATUS(stat) caen_board_acq_status(stat)
#define DEVICE_BLT_READ(evtp) caen_blt_read(evtp)
#endif

//Define test stub functions
CAENComm_ErrorCode STUBComm_OpenDevice (CAENComm_ConnectionType LinkType, int LinkNum, int ConetNode, uint32_t VMEBaseAddress, int *handle);
CAENComm_ErrorCode STUBComm_CloseDevice(int handle);
CAENComm_ErrorCode STUBComm_Read32(int handle, uint32_t Address, uint32_t *Data);
CAENComm_ErrorCode STUBComm_Write32(int handle, uint32_t Address, uint32_t Data);
CAENComm_ErrorCode STUBComm_SWRelease(char *SwRel);
CAENComm_ErrorCode STUBComm_AcqStatus(acq_stat *stat);

#ifdef  TARGET_DEVICE_STUB
#define DEVICE_OPEN(LinkType, LinkNum, ConetNode, VMEBaseAddress, handle) STUBComm_OpenDevice (LinkType, LinkNum, ConetNode, VMEBaseAddress, handle)
#define DEVICE_CLOSE(handle) STUBComm_CloseDevice(handle)
#define DEVICE_SW_VER(SwRel) STUBComm_SWRelease(SwRel)
#define DEVICE_REG_READ32(handle, Address, Data)  STUBComm_Read32(handle, Address, Data)
#define DEVICE_REG_WRITE32(handle, Address, Data) STUBComm_Write32(handle, Address, Data)
#define DEVICE_ACQ_STATUS(stat) STUBComm_AcqStatus(stat)
#define DEVICE_BLT_READ(evtp) STUB_blt_read(evtp)
#endif 

//------------------------------------------------------------------------------
//board communication

//Caen DT5742 digitized signal geometry, (8ch+1tr)*2gr
//digitizer event header
typedef struct evt_hdr { //3*DW=12B
    uint32_t evt_size:   32; //total evt size, 27b
    uint16_t samples:    16; //samples pre channel, 12b
    uint32_t evt_ttag:   32; //evt time tag, 1+31b
    uint32_t evt_contr:  24; //evt counter, 24b
    uint8_t grp_mask  :   4; //evt group mask, 2b
    uint8_t board_fail:   1; //board PLL failm 1b
    uint8_t ttag_owflw:   1; //evt time tag overflow, 1b
} caen5742_evt_header;

//digitizer group header
typedef struct grp_hdr { //1.5*DW=6B
    uint32_t gpr_ttime  :32; //group trigger time, 30b
    uint16_t start_cell :10; //start index cell, 10b=1024
    uint16_t samples    :16; //samples per ch
    uint8_t  dgtz_freq   :2; //group dgtz ferquency, 2b
    uint8_t  tr_dgtzd    :1; //is trigger digitized, 1b
} caen5742_grp_header;

//ein event
typedef struct evt_str {
    struct _timeb tb; //timestamp          
    uint32_t odo_tot; //total odometer, 32b
    uint32_t odo_run; //last run odometer, 32b

    caen5742_evt_header evt_hdr;
    caen5742_grp_header gr_hdr[CAEN_DT7542_GR_NUM];
    uint16_t           evt_buf[CAEN_DT7542_GR_NUM][CAEN_DT7542_CH_NUM+1][CAEN_DT7542_SIGWIN];
} caen5742_evt;

//local via USB or connet
int caen_board_loc_open  (const int link_num, const int conet_node, char *sres);
int caen_board_loc_close (char *sres);
int caen_board_loc_ver   (char *sver,char *sres);
int caen_board_reg_init  (char *sres);

typedef void (*data_callback) (const caen5742_evt *evt);
void caen_board_loop(uint8_t *stop, data_callback);

extern acq_stat caen_board_status;
int16_t caen_board_acq_status(acq_stat *stat);
int8_t caen_blt_read (caen5742_evt *evtp);

int8_t mem_buf_get (caen5742_evt **evtp, const uint32_t pos);

int8_t fwrite_evt(caen5742_evt *evtp);
int8_t fload_evt( const char* fname);

//READ register by name
int8_t caen_reg_read_name(  //ret   - return error codes, ok=0, see caen_err
    const char *name,       //name  - command or setting text name, see caen_reg_map
    const uint8_t gr,       //gr    - group number, if required by command
    const uint8_t ch,       //ch    - channel number, if required by command
    uint32_t *ival,         //*ival - return value in integer format, dump value
    char *sval,             //*sval - return value as string, parsed and formatted
    char *sres);            //*res  - string repesentation of request for log

//WRTIE register by name
int8_t caen_reg_write_name( //ret   - return error codes, ok=0, see caen_err
    const char *name,       //name  - command or setting text name, see caen_reg_map
    const uint8_t gr,       //gr    - group number, if required by command
    const uint8_t ch,       //ch    - channel number, if required by command
    uint32_t ival,          //ival  - set value as integer format
    char *sval,             //*sval - set value as string, replace int, 2DO
    char *sres);            //*res  - string repesentation of request for log

#endif //__CAEN5742_H