/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * caen5742.c
 * Caen digitizer low level driver */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "caen5742.h"

//------------------------------------------------------------------------------
//BOARD global state

int caenh;  //main CAEN device handler

//main events memory buffer
#define MEM_BUF_EVT_SZ 1000
caen5742_evt mem_buf [MEM_BUF_EVT_SZ];
uint16_t buf_curW;  //mem buf cursor write
uint16_t buf_curR;  //mem buf cursor read
uint32_t odo_tot;   //event odometer since start
uint32_t odo_run;   //event odometer since last run

uint8_t  is_run_cmd;          //dgtzr run
acq_stat caen_board_status;   //board acquisition status
#define CAEN_LAZY_SLEEP 10000 //not run mode
#define CAEN_FAST_SLEEP 1     //run mode


//Enums and errors
//caen comm error int-to-string, max 50 chars here
char* caen_err2str(int err){
    switch (err){
    case CAENComm_Success:                  return "ok";          break;
    case CAENComm_VMEBusError:              return "VME bus error during the cycle"; break;
    case CAENComm_CommError:                return "Communication error";    break;
    case CAENComm_GenericError:             return "Unspecified error";   break;
    case CAENComm_InvalidParam:             return "Invalid parameter err";   break;
    case CAENComm_InvalidLinkType:          return "Invalid Link Type err";   break;
    case CAENComm_InvalidHandler:           return "Invalid handler err";   break;
    case CAENComm_CommTimeout:              return "Communication timeout err";   break;
    case CAENComm_DeviceNotFound:           return "Unable to Open the requested Device err";  break;
    case CAENComm_MaxDevicesError:          return "Maximum number of devices exceeded err";   break;
    case CAENComm_DeviceAlreadyOpen:        return "The device is already opened err";         break;
    case CAENComm_NotSupported:             return "Not supported function err";   break;
    case CAENComm_UnusedBridge:             return "There aren't board controlled by that CAEN Bridge err";  break;
    case CAENComm_Terminated:               return "Communication terminated by the Device err";             break;
    case CAENComm_UnsupportedBaseAddress:   return "The Base Address is not supported err, desktop device?"; break;

    //case drv_err_ok:                        return "ok"; break; //goto CAENComm_Success
    case drv_err_not_found:                 return "reg name not found"; break;
    case drv_err_ro:                        return "read only reg";  break;
    case drv_err_wo:                        return "write only reg"; break;
    case drv_err_spi_busy:                  return "spi is busy before wr"; break;

    default: return "an error"; break;
    }
}

//------------------------------------------------------------------------------
//STUB CAENcomm simulator block for test
#ifdef TARGET_DEVICE_STUB
acq_stat dbg_astat; //debug acq state

void STUB_RegMapInit();
CAENComm_ErrorCode STUBComm_Write32(int handle, uint32_t Address, uint32_t Data);

CAENComm_ErrorCode STUBComm_OpenDevice (CAENComm_ConnectionType LinkType, int LinkNum,
                                        int ConetNode, uint32_t VMEBaseAddress, int *handle){
    caenh = 0x1;
    dbg_astat.board_rdy=1;
    dbg_astat.pll_unlck=1;
    srand(time(NULL));
    STUB_RegMapInit();

    fload_evt("raw/0.csv");
    buf_curR = 0;

    return CAENComm_Success;
}

CAENComm_ErrorCode STUBComm_CloseDevice(int handle){
    dbg_astat.acq_run  =0;
    dbg_astat.pll_unlck=0;
    dbg_astat.board_rdy=0;
    caenh = 0x0;
}

uint8_t trg_simu;

CAENComm_ErrorCode STUBComm_AcqStatus(acq_stat *stat){
    if (caenh){
        memcpy(stat, &dbg_astat, sizeof(acq_stat));
        if (dbg_astat.acq_run){
            dbg_astat.trg_in   = (++trg_simu%15)?0:1;
            dbg_astat.evt_rdy  = (rand()%24>0)?1:0;
            dbg_astat.evt_full = (rand()%16>0)?0:1;
        }

        return CAENComm_Success;
        }
    else return CAENComm_CommError;
}

CAENComm_ErrorCode STUBComm_SWRelease(char *SwRel){
    if (caenh){
        strcpy(SwRel,"1.2.3");
        return CAENComm_Success;
        }
    else return CAENComm_CommError;
}

//debug miltichannel signal generator
uint16_t sigar[CAEN_DT7542_CT_NUM][CAEN_DT7542_SIGWIN];
void have_a_sigar(){
     for (int k=0; k<CAEN_DT7542_CT_NUM; k++){
        uint16_t dxs = rand()%50;
        for (int i=0; i<CAEN_DT7542_SIGWIN; i++){
                if (i<400+dxs)
                sigar[k][i] = 160-5*k+rand()%40;
            else if (i<500+dxs)
                sigar[k][i] = 160-5*k+rand()%200  + (i-400-dxs)*(CAEN_DT7542_ADCRES-400)/100;
            else if (i<700+dxs)
                sigar[k][i] = 160-8*k+rand()%80  + (CAEN_DT7542_ADCRES-400);
            else if (i<800+dxs)
                sigar[k][i] = 160-8*k+rand()%200 + (CAEN_DT7542_ADCRES-400) - (i-700-dxs)*(CAEN_DT7542_ADCRES-400)/100;
            else if (i<CAEN_DT7542_SIGWIN)
                sigar[k][i] = 160-5*k+rand()%20;
        }
     }
}

#define DATA_FILE
//#define DATA_RND

int STUB_blt_read(caen5742_evt *evtp){

#ifdef DATA_FILE
    //evtp=&mem_buf[buf_curR++];
    if (++buf_curR>25)buf_curR=0;
#endif //DATA_FILE

#ifdef DATA_RND
    if (dbg_astat.acq_run) have_a_sigar();
    for (int g=0; g<CAEN_DT7542_GR_NUM; g++)
        for (int c=0; c<CAEN_DT7542_CH_NUM+1; c++)
            for (int i=0; i<CAEN_DT7542_SIGWIN; i++)
            evtp->evt_buf[g][c][i]=sigar[c][i];
#endif// DATA_RND
}

//STUB reg map simulator
struct ein_reg{
    uint16_t addr;
    uint32_t val;
};

#define STUB_MAP_SIZE 100
struct ein_reg stub_reg_map[STUB_MAP_SIZE]; //BAD shure 100 is enough
uint8_t stub_reg_map_cur; //enf of array

void STUB_RegMapInit(){
    STUBComm_Write32(caenh,CAEN_DGTZ_CHANNEL_AMC_FPGA_FW_BASE_ADDRESS,123124);
    STUBComm_Write32(caenh,CAEN_DGTZ_FW_REV_ADD,34535);
    STUBComm_Write32(caenh,CAEN_DGTZ_BOARD_INFO_ADD,0);
    STUBComm_Write32(caenh,CAEN_DGTZ_DRS4_FREQUENCY_REG,456546);
    STUBComm_Write32(caenh,CAEN_DGTZ_X742_DRS4_TEMP_BASE_ADDRESS,166);
    STUBComm_Write32(caenh,CAEN_DGTZ_BLT_EVENT_NUM_ADD,45634);
    STUBComm_Write32(caenh,CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD,6876);
    STUBComm_Write32(caenh,CAEN_DGTZ_CUSTOM_SIZE_REG,734);
    STUBComm_Write32(caenh,CAEN_DGTZ_BROAD_CH_CTRL_ADD,0xffffffff);
    STUBComm_Write32(caenh,CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD,456456);
    STUBComm_Write32(caenh,CAEN_DGTZ_POST_TRIG_BASE_ADDRESS,456);
    STUBComm_Write32(caenh,CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD,3453);

    STUBComm_Write32(caenh,CAEN_DGTZ_ACQ_CONTROL_ADD,0);
    STUBComm_Write32(caenh,CAEN_DGTZ_SW_TRIGGER_ADD,0);
    //STUBComm_Write32(caenh,);
}

CAENComm_ErrorCode STUBComm_Read32(int handle, uint32_t Address, uint32_t *Data){
    if (caenh){

        if (Address == CAEN_DGTZ_REG_WO) return drv_err_wo;

        if (Address == CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS ||
            Address == CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS)
            return CAENComm_Success;
        
        uint8_t i=0;
        for (i=0; i<STUB_MAP_SIZE; i++){
            if (stub_reg_map[i].addr == Address) {
                *Data = stub_reg_map[i].val; 
                break;
                }
            }
        
        PRN_DIAG(" stubr %04x=0x%08x\n", Address, *Data);

        if (i==STUB_MAP_SIZE) return CAENComm_InvalidParam;
                         else return CAENComm_Success;
        }
    else return CAENComm_CommError;
}

CAENComm_ErrorCode STUBComm_Write32(int handle, uint32_t Address, uint32_t Data){
    
    if (caenh){
        
        if (Address == CAEN_DGTZ_REG_RO) return drv_err_ro;
        if (Address == CAEN_DGTZ_ACQ_CONTROL_ADD && (Data & (1UL<<2)) == (1UL<<2) ) dbg_astat.acq_run = 1;
        if (Address == CAEN_DGTZ_ACQ_CONTROL_ADD && (Data & (1UL<<2)) == (0UL)    ) {dbg_astat.acq_run = 0; dbg_astat.evt_rdy = 0;}

        uint8_t i=0;
        for (i=0; i<STUB_MAP_SIZE; i++)
            if (stub_reg_map[i].addr == Address) stub_reg_map[i].val=Data; 
        
        if (i==STUB_MAP_SIZE){
            PRN_DIAG (" add_new pos=%d, %04x=%d\n",stub_reg_map_cur, Address, Data);
            stub_reg_map[stub_reg_map_cur].addr=Address;
            stub_reg_map[stub_reg_map_cur].val =Data;
                         stub_reg_map_cur++;
        }
        PRN_DIAG(" stubw %04x=0x%08x\n", Address, Data);
        return CAENComm_Success;
        }
    else return CAENComm_CommError;
}
#endif //TARGET_DEVICE_STUB

//------------------------------------------------------------------------------
//BOARD COMM

//LinkType:CAENComm_USB, LinkNum=0, ConetNode=0, VMEBaseAddress=0
int caen_board_loc_open(const int link_num, const int conet_node, char *sres){
    int err = DEVICE_OPEN(CAENComm_USB,link_num,conet_node,0,&caenh);
    snprintf (sres, SRES_MAX_LEN, "board_open:%d.%d %s", link_num, conet_node, caen_err2str(err));
    return err;
}

//close handler
int caen_board_loc_close(char *sres){
    int err = DEVICE_CLOSE(caenh);  
    snprintf (sres, SRES_MAX_LEN, "board_close %s, tot=%d, run=%d\n",caen_err2str(err), odo_tot, odo_run);
        memset(&caen_board_status,0,sizeof(acq_stat));
    return err;
}

//SW release ver
int caen_board_loc_ver(char *sver,char *sres){
    int err = DEVICE_SW_VER(sver);
    snprintf (sres, SRES_MAX_LEN, "board_ver %s = %s",caen_err2str(err),sver);
    return err;
}

//some hardcoded board init
//2DO - place to XML
int caen_board_reg_init(char *sres){
    int err=0;
    uint32_t val;
    
    err += CAENComm_Read32 (caenh, CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, &val); //GPO soft mode
    err += CAENComm_Write32(caenh, CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, val|(1UL<<15));

    err += CAENComm_Write32(caenh, CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_ADD, 32768);//36700); //ftr_dc
    err += CAENComm_Write32(caenh, CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_ADD, 20934); //ftr_tr

    //err += CAENComm_Write32(caenh, CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS, 32768);//36700); //ftr_dc
    //err += CAENComm_Write32(caenh, CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_BASE_ADDRESS, 20934); //ftr_tr
    
    err += CAENComm_Write32(caenh, CAEN_DGTZ_CH_ENABLE_ADD, 0x3); //gr_en

    //[12]TRn enable, Reserved MUST: [10,9]=0, [8]=1, [4]=1
    err += CAENComm_Write32(caenh, CAEN_DGTZ_BROAD_CH_CLEAR_CTRL_ADD, 0xFFFF);
    //err += CAENComm_Write32(caenh, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, 0x0110); //tr_nodig
    err += CAENComm_Write32(caenh, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, 0x0910); //tr_dig
    
    snprintf(sres, SRES_MAX_LEN, "Pre init %s", (err)?"err":"ok");
}

void caen_board_loop(uint8_t *stop, data_callback dc ){
    printf(" board_loop_start\n");
    srand(time(NULL));
       
    odo_tot = 0;
    odo_run = 0;
    struct _timeb tb1, tb2;
    uint32_t dt;
    uint32_t odo1;

    uint16_t tc=0; //thin out
    uint16_t slp=CAEN_LAZY_SLEEP;

        //_ftime(&tb2);
        //dt = (uint32_t) (1000.0 * (tb2.time - tb1.time) + (tb2.millitm - tb1.millitm));
        //if (dt>1000) { printf(" -%devt/s- ", odo_tot-odo1); tb1=tb2; odo1=odo_tot; }

    while(*stop){
        DEVICE_ACQ_STATUS(&caen_board_status);
        
        if (caen_board_status.evt_rdy){
            DEVICE_BLT_READ( (&mem_buf[buf_curR]) );
            dc(&mem_buf[buf_curR]);
        }
        
        if ( is_run_cmd && slp==CAEN_LAZY_SLEEP) slp=CAEN_FAST_SLEEP;
        if (!is_run_cmd && slp==CAEN_FAST_SLEEP && !caen_board_status.evt_rdy) slp=CAEN_LAZY_SLEEP;
        
        for ( uint32_t s = slp*30000; s; --s ) __asm__("nop"); //3Ghz=3000*x(usec) 
        //Sleep(slp); //printf(".");
    }   
    //while(caen_acq_stat()) caen_btl_read(&mem_buf[0]);

    printf("board_loop_close\n");
}

//return event from mem buffer
int8_t mem_buf_get (caen5742_evt **evtp, const uint32_t pos){
    if (pos>MEM_BUF_EVT_SZ) return drv_err_mem_outr;
    *evtp=&mem_buf[pos];
}


//------------------------------------------------------------------------------
//BLT READ

//get acq status
int16_t caen_board_acq_status(acq_stat *stat){
    uint32_t data32=0;
    int16_t ret = CAENComm_Read32 (caenh, CAEN_DGTZ_ACQ_STATUS_ADD, &data32);
    if (ret == CAENComm_Success){
        stat->acq_run   = (data32&(1UL<<2))>>2;
        stat->evt_rdy   = (data32&(1UL<<3))>>3;
        stat->evt_full  = (data32&(1UL<<4))>>4;
        stat->pll_unlck = (data32&(1UL<<7))>>7;
        stat->board_rdy = (data32&(1UL<<8))>>8;
        stat->trg_in    = (data32&(1UL<<16))>>16; //! not works

        /*printf(" %s-%s-%s-%s-", (stat->board_rdy)?"b":".", (stat->acq_run)?"run":".",
                                (stat->evt_rdy)?"evt":".", (stat->evt_full)?"evf":".");
        CAENComm_Read32 (caenh, 0x814C, &data32); printf("es%d ", data32);
        CAENComm_Read32 (caenh, 0x1088, &data32); printf("gns%x ", data32);*/
    }
    return ret;
}

void fwrtie_evt(caen5742_evt *evt_str); //dbg

void tb2str(const struct _timeb tb){
    char buffer[26];  
    strftime(buffer, 26, "%j.%H%M%S", localtime((time_t*)&tb));
    printf("%s.%d\n", buffer, tb.millitm);    
        //char *timeline = ctime( & ( tb.time ) );
        //printf( "The time is %.19s.%hu %s", timeline, tb.millitm, &timeline[20] );
}


//read header and payload
//DW sizes 3078 3462 6152 6920
int8_t caen_blt_read(caen5742_evt *evtp){
    int err;
  
    _ftime( &evtp->tb ); //local system timestamp
    //tb2str( evtp->tb );

    memset(evtp,0,sizeof(caen5742_evt));

    uint32_t hdr[4];
    memset(hdr,0,4*CAEN_DT7542_HDR_SZ);
    uint32_t nw;
        
    err = CAENComm_BLTRead(caenh, 0, hdr, CAEN_DT7542_HDR_SZ, &nw);
        
        //printf(" btl_hdr ret=%d, nw=%d, sz=%d, cnt=%d, hdr0=0x%x\n",err,nw,
        //    (hdr[0]&(0xFFFFFFF)), hdr[2]&(0xFFFFFF), hdr[0] );

        if (err!= CAENComm_Success) return err;
        if (nw != CAEN_DT7542_HDR_SZ)        return drv_err_blt_hdr_sz;
        if ((hdr[0]&(0xF<<28)) != (0xA<<28)) return drv_err_blt_hdr_cor;

        evtp->evt_hdr.evt_size   = hdr[0]&(0xFFFFFFF);
        evtp->evt_hdr.evt_ttag   = hdr[3]&(0x7FFFFFFF);
        evtp->evt_hdr.evt_contr  = hdr[2]&(0xFFFFFF);
        evtp->evt_hdr.grp_mask   = hdr[1]&(0x03);
        evtp->evt_hdr.board_fail = hdr[1]&(0x01<<26);
        evtp->evt_hdr.ttag_owflw = hdr[3]&(0x01<<31);

    uint32_t evt_sz = (hdr[0]&(0xFFFFFFF))-4; //evt size, lw
    if (!evt_sz)    return drv_err_blt_zero;

    uint32_t buff[7000];
    memset(buff,0,sizeof(buff));

    err = CAENComm_BLTRead(caenh, 0, buff, evt_sz, &nw);
    //printf(" btl_bdy ret=%d, nw=%d, sz=%d, gm=%d, tr=%d, hdr=0x%x\n",
    //        err,nw,evt_sz, hdr[1]&0x03, ((buff[0]&0x1000)>>12), buff[0]);

    if (err!= CAENComm_Success) return err;  //=> here is read start
    
    //signal geometry calc
    uint16_t esz= nw;                     //event size in DoubleWords, no header
    uint8_t  gedsz = 4;  //gr evt desc sz
    uint8_t  gttsz = 4;  //gr tr tile sz
    uint8_t  tm = (buff[0]& 0x1000)>>12;  //tr0 trigger mask 0,1
    uint8_t  gm = hdr [1]& 0x03;          //group mask 00,01,10,11=both
    uint8_t  gc = (gm&0x1) + ((gm&0x2)>>1UL); //group count 0-1-2
    
    uint16_t rldw = (esz - 2*gc)/ ((8+tm) * gc); //record length in DW, 0x8020
    uint16_t rlsm = rldw*32/12;                  //in 12b samples
    evtp->evt_hdr.samples = rlsm; //each ch rec length in 12b samples, calculated
    
    uint16_t gsdw = gedsz/4 + rldw*(8+tm) + gttsz/4;      //group shift in dw

    //printf(" esz=%d, gsdw=%d, rlsm=%d, rldw=%d\n", esz, gsdw, rlsm, rldw);

    //fill gr evt desc
    for (uint8_t g=0; g<gc; g++){
        uint8_t rg = (gc==1)? gm-1 : g ; //real group 0,1,0-1
        evtp->gr_hdr[rg].start_cell = (buff[0+gsdw*g] & 0x3FF00000) >>20;
        evtp->gr_hdr[rg].dgtz_freq  = (buff[0+gsdw*g] & 0x30000)    >>16;
        evtp->gr_hdr[rg].tr_dgtzd   = (buff[0+gsdw*g] & 0x1000)     >>12;
        evtp->gr_hdr[rg].samples    = (buff[0+gsdw*g] & 0xFFF); //from evnt_desc
        evtp->gr_hdr[rg].gpr_ttime  = (buff[gsdw*(g+1)-1] & 0x3FFFFFFF);
          //printf(" gr%d: freq=%d, tr=%d, chsz=%d, ttime=%d \n", rg,
            //evtp->gr_hdr[rg].dgtz_freq,evtp->gr_hdr[rg].tr_dgtzd, 
            //buff[0+gsdw*g] & 0xFFF, evtp->gr_hdr[rg].gpr_ttime);
    }
    
    //uint16_t caen_evt_buf[CAEN_DT7542_GR_NUM][CAEN_DT7542_CH_NUM+1][CAEN_DT7542_SIGWIN];
    
    uint8_t *buff8 = (uint8_t*) buff; //8 bit representation of 32b buffer
    uint16_t lb,rb,trv; //left right data from 12+12=24=8*3, trig value
    uint8_t  rg;    //real group
    uint16_t as;    //address of 8bit source buffer
    uint16_t ad;    //address of 16bit dest buffer
    uint8_t chn = CAEN_DT7542_CT_NUM;       //channels total

    for (uint16_t i=0; i<rlsm; i++){

        for (uint8_t g=0; g<gc; g++){
            rg = (gc==1)? gm-1 : g ; //real group 0,1,0-1

            for (uint8_t k=0; k<4; k++){ //4*(12+12=8b*3) blocks
                as = gedsz + i*12+k*3 + gsdw*4*g; //4b evt_desc + shift + grp_shift
                rb= (buff8[as]) + ((buff8[as+1]&0x0F)<<8);     //right 12b
                lb=((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4); //left 12b
                evtp->evt_buf[rg][k*2  ][i]=rb;
                evtp->evt_buf[rg][k*2+1][i]=lb;
            }

            if (gm){ //trigga
                as= gedsz + 12*rlsm + (i>>1)*3 + gsdw*4*g;   //TR data at the tail
                if (!i%2) trv = (buff8[as]) + ((buff8[as+1]&0x0F)<<8);
                    else  trv =((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4);
                evtp->evt_buf[rg][8][i]=trv; //9th channel is a trigger
            }
        }
    }
    //insert and update odometers
    evtp->odo_tot = odo_tot; odo_tot++;
    evtp->odo_run = odo_run; odo_run++;

    //fwrtie_evt(evtp); //Careful with That Axe, Eugene
}

//print data to csv file, 18ch columns, evt_blt rows, uint16_t, leading zero
int8_t fwrite_evt(caen5742_evt *evtp){

    FILE *fptr;
    char fname[32], sbuf[32];
    strftime(sbuf, 26, "%j.%H%M%S", localtime((time_t*)&evtp->tb));
    sprintf(fname,"csv/caen-%s.%3d.csv", sbuf, evtp->tb.millitm);
    fptr = fopen(fname,"w");
    printf("file %s", fname);
    if (fptr) {
        //fprintf(fptr,"smpl=%04d gm=%d\n", evt_str->evt_hdr.samples, evt_str->evt_hdr.grp_mask);
        for (uint16_t i=0; i<evtp->evt_hdr.samples; i++){
            for (uint16_t g=0; g<2; g++)
                for (uint16_t c=0; c<9; c++)
                    fprintf(fptr,"%04d ", evtp->evt_buf[g][c][i]);
            fprintf(fptr,"\n");
        }
    fclose(fptr);
    printf(" ok\n");
    } else printf(" err\n");
}

//load data file in to mem_buf, text (csv) or binary (bin)
//text: 4digit+space * (8+1+8+1=18) channels, CL/LR, 1024 lines per laser shot
//binary: 2bytes * (8+1+8+1=18) channels, 1024 "lines" per laser shot
int8_t fload_evt( const char* fname){

    uint8_t txtbin; //dat or binary
    char *fext = strrchr(fname, '.');
         if ( fext && fext != fname && !strcmp(fext+1,"csv") ) txtbin = 1;
    else if ( fext && fext != fname && !strcmp(fext+1,"bin") ) txtbin = 0;
    else {printf("%s - ext must be bin or csv\n", fname); return -1;}

    printf("file=%s, ", fname);
    FILE *fptr = fopen(fname,"rb");
    if (!fptr) {printf("err open\n"); return -2;}

    fseek (fptr , 0L , SEEK_END);
    uint32_t fsz = ftell (fptr); //file size bytes
    rewind (fptr);

    //file size samples_lines total (samples * laser_pulses)
    //txt_line: 4 digit (12b=4096 max) + space, 5 symb per sample, 8+1+8+1 chanels, +CL/LR = 92 B
    //bin_line: 8+1+8+1 *2 B each sample
    uint32_t fszl = (txtbin)? fsz/( (4+1)*18 +2 ) : fsz/(18*2); 

    //file size lazer pulses, fixed 1024 samples per each pulse
    uint32_t fszp = fszl/1024;

    printf ("type=%s, sz=%db/%ds/%dp. ", (txtbin)?"txt":"bin", fsz, fszl, fszp);

    //whole file buffer
    char *fbuf = (char*) malloc (sizeof(char)*fsz); 
    if (!fbuf) {printf("err malloc\n"); return -3;}
    
    size_t res = fread (fbuf, 1, fsz, fptr);
    if (res != fsz) {printf("err %zu read\n", res); free (fbuf); return -4;}

    uint32_t cur=0; uint16_t x;
    for (int p=0; p<fszp; p++){
        for (int i=0; i<CAEN_DT7542_SIGWIN; i++){ //fix window = 1024
            for (int g=0; g<CAEN_DT7542_GR_NUM; g++){
                for (int c=0; c<CAEN_DT7542_CH_NUM+1; c++){
                    mem_buf[p].evt_buf[g][c][i] = (txtbin)? atoi(fbuf+cur) : (*((uint16_t *)(fbuf+cur)));
                    cur+=((txtbin)?5:2);
                }
            }
            if (txtbin) cur+=2; // "/n"
        }
        _ftime( &mem_buf[p].tb);
        mem_buf[p].odo_tot = p+113;
        mem_buf[p].odo_run = p;
    }
       
    fclose(fptr);
    free (fbuf);
    printf("\nParce %d ok\n", cur);
}

//------------------------------------------------------------------------------
//Reg read-write with slight parsing

//parse specific caen dgtzr data structures for some commands
void caen_reg_parcer(const uint32_t addr, const uint32_t ival, char *sval){
    //printf("%x %d",addr,ival);
    switch (addr){

    case CAEN_DGTZ_CHANNEL_AMC_FPGA_FW_BASE_ADDRESS: //ver_amc
         sprintf (sval, "%d.%02d",// %02x.%02d.%02d", //too long
            (ival>>8 &0xFF),
            (ival>>0 &0xFF));
            //(ival>>16&0xFF),
            //(ival>>24&0xF ),
            //(ival>>28&0xF ));
    break;

    case CAEN_DGTZ_FW_REV_ADD: //ver_roc
        sprintf (sval, "%d.%02d",// %02x.%02d.%02d", //too long
            (ival>>8 &0xFF),
            (ival>>0 &0xFF));
            //(ival>>16&0xFF),
            //(ival>>24&0xF ),
            //(ival>>28&0xF ));
    break;

    case CAEN_DGTZ_BOARD_INFO_ADD:
        sprintf (sval, "%sf %se %sg", //memory config
            ((ival>>0 &0xFF) == 6)?"742":"unk_",
            ((ival>>8 &0xFF) == 1)?"128":"1024",
            ((ival>>16&0xFF) == 2)?"2":"4"
            );
    break;

    case CAEN_DGTZ_X742_DRS4_TEMP_BASE_ADDRESS: //drs4 temp
        sprintf (sval, "%d", (int8_t)ival);     // ‐64°C,0xC0,1100 000; 127°C,0x7F, 0111 1111
    break;

    default:
    snprintf(sval,SVAL_MAX_LEN, "%d", ival);  //default int_to_string[16]
    break;
    }
}

//READ register by name
int8_t caen_reg_read_name(  //ret   - return error codes, ok=0, see caen_err
    const char *name,       //name  - command or setting text name, see caen_reg_map
    const uint8_t gr,       //gr    - group number, if required by command
    const uint8_t ch,       //ch    - channel number, if required by command
    uint32_t *ival,         //*ival - return value in integer format, dump value
    char *sval,             //*sval - return value as string, parsed and formatted
    char *sres)             //*res  - string repesentation of request for log
{
    if (!strcmp(name,"ver_drv"))
        return caen_board_loc_ver (sval,sres);

    for (uint8_t i=0; i<sizeof(caen_reg_map)/sizeof(caen_reg_addr); i++){ //try find reg_addr by name
        
        if (!strcmp(caen_reg_map[i].name, name)){
            caen_reg_addr ra = caen_reg_map[i];
            
            uint32_t addr=ra.addrR; //read addr
            int8_t   ret_err;       //return error
            uint32_t ret_val;       //return value

            snprintf(sres, SRES_MAX_LEN, "reg rd %s %04X", ra.name, addr);

            //Write only reg
            if (addr == CAEN_DGTZ_REG_WO){
                //2DO //int err=(caenh)?drv_err_wo:CAENComm_InvalidHandler; //check if board open
                ret_err = drv_err_wo;
                snprintf(sres, SRES_MAX_LEN, "%s err=%s", sres, caen_err2str(ret_err));
                return ret_err;
                }

            //0x10A4 set Channel selection reg 0x10A4 before read
            if (addr == CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS || //if 0x8080 Group n Channel Threshold
                addr == CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS){        //or 0x8098 Group n Channel DC Offset

                ret_err = DEVICE_REG_WRITE32 (caenh,
                    CAEN_DGTZ_CHANNEL_SELECTION_ADD | ((gr&0x3)<<8),
                    ch & UINT04_MAX);
                
                if (ret_err!=CAENComm_Success){
                    snprintf(sres, SRES_MAX_LEN, "reg ch_sel err=%s", caen_err2str(ret_err));
                    return ret_err;
                }
            }
            
            //mask group, for channel based reg 0x10xx
            if ( (addr&0xFF00) == 0x1000){
                addr |= ((gr&0x3)<<8);  //set ch number
                snprintf(sres, SRES_MAX_LEN, "%s:%d.%d", sres, gr, ch); //add 2 log
                }

            ret_err = DEVICE_REG_READ32(caenh, addr, &ret_val);

                if (ret_err==CAENComm_Success){
                        PRN_DIAG(" rd %s,%04x=%04x\n", name, addr, ret_val);
                        
                        uint8_t mshift = log2(ra.mask & -ra.mask);  //if rightmost set bit of mask >0
                        if (mshift) {
                            *ival = ret_val & ra.mask;   //mask raw value
                            *ival >>= mshift ;           //shift value to the right
                            }
                            else  
                            *ival = ret_val & ra.mask;   //already right aligned
                        
                        caen_reg_parcer(addr, *ival, sval); //convet to string
                        snprintf(sres, SRES_MAX_LEN, "%s=%x,%s", sres, *ival, sval);
                        if (*ival != ret_val) snprintf(sres, SRES_MAX_LEN, "%s 0x%x", sres, ret_val);
                        return drv_err_ok;
                }else{ //comm_read error
                    snprintf(sres, SRES_MAX_LEN, "%s err=%s", sres, caen_err2str(ret_err));
                    return ret_err; //caen_comm_read error
                }
            }
    }
    //string name for register dndt exist
    snprintf(sres,SRES_MAX_LEN, "reg rd %s err=%s", name, caen_err2str(drv_err_not_found));
    return drv_err_not_found;  //register name not found
}

//WRTIE register by name
int8_t caen_reg_write_name( //ret   - return error codes, ok=0, see caen_err
    const char *name,       //name  - command or setting text name, see caen_reg_map
    const uint8_t gr,       //gr    - group number, if required by command
    const uint8_t ch,       //ch    - channel number, if required by command
    uint32_t ival,          //ival  - set value as integer format
    char *sval,             //*sval - set value as string, replace int, 2DO
    char *sres)             //*res  - string repesentation of request for log
{
    for (uint8_t i=0; i<sizeof(caen_reg_map)/sizeof(caen_reg_addr); i++){ //try find reg_addr by name
        caen_reg_addr ra = caen_reg_map[i];

        if (!strcmp(caen_reg_map[i].name, name)){
            caen_reg_addr ra = caen_reg_map[i];

            uint32_t addr=ra.addrW; //write addr
            int8_t   ret_err;       //return error
            uint32_t ret_val;       //return value
            
            uint32_t val32 = ival;
            if (sval) val32=atoi(sval); //BAD overwrite integer if string value passed

            //calback to polling
            if (!strcmp(ra.name,"acq_run")) {
                is_run_cmd = val32; //set run mode for fast polling
                if (val32) odo_run = 0; //drop run odo
                }
            if (!strcmp(ra.name,"tr_soft_kick")) caen_board_status.trg_in = 1; //tr led, not good

            snprintf(sres, SRES_MAX_LEN, "reg wr %s %04X", ra.name, addr);

            //Read only reg
            if (addr == CAEN_DGTZ_REG_RO) {
                snprintf(sres, SRES_MAX_LEN, "%s err=%s", sres, caen_err2str(drv_err_ro));
                return drv_err_ro;
                }
           
            //0x1n88 check if spi busy, bit[2]=0
            if (addr == CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS ||
                addr == CAEN_DGTZ_CHANNEL_DAC_ADD          ||
                addr == CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_BASE_ADDRESS ||
                addr == CAEN_DGTZ_GROUP_FASTTRG_THR_V1742_ADD          ||
                addr == CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS ||
                addr == CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_ADD){

                ret_err = DEVICE_REG_READ32(caenh, 0x1088 | ((gr&0x3)<<8), &ret_val);
                    PRN_DIAG(" spi 0x%08x\n",ret_val);
                    if (ret_err!=CAENComm_Success){
                        snprintf(sres,SRES_MAX_LEN, "reg chk spi %s",caen_err2str(ret_err));
                        return ret_err;
                    }
                    if (ret_err&0x4){ //bit[2] not ready
                        ret_err = drv_err_spi_busy;
                        snprintf(sres,SRES_MAX_LEN, "reg chk spi %s",caen_err2str(ret_err));
                        return ret_err;
                    }
                }

            //0x8004 (BitSet) -> 0x8008 (BitClear)
            if (addr == CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD     //if adddr= Board Configuration 
                && val32 ==0 ){                                  //and if need to clear bit
                addr = CAEN_DGTZ_BROAD_CH_CLEAR_CTRL_ADD;        //change addr to clear_bit_reg
                val32 = 1;                                       //set bit to 1
                }
            
            //read-mask-write
            // 8100; 810c; 8110; 811c; EF00
            uint8_t mshift = log2(ra.mask & -ra.mask);  //calc rightmost set bit of mask
            if (mshift) val32 <<= mshift;  //shift if from middle
                        val32 &=  ra.mask; //mask for right aligment only

            if (addr == CAEN_DGTZ_ACQ_CONTROL_ADD ||
                addr == CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD ||
                addr == CAEN_DGTZ_FP_TRIGGER_OUT_ENABLE_ADD ||
                addr == CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD ||
                addr == CAEN_DGTZ_VME_CONTROL_ADD){
                
                if (!DEVICE_REG_READ32(caenh,addr,&ret_val))     //get old val
                    if (val32>>mshift)
                               val32 = ret_val |  (1UL<<mshift); //set bit
                          else val32 = ret_val & ~(1UL<<mshift); //clear bit
                //printf(" mask %x %x\n",ret_val, val32);
            }           

            
            //0x1n80, add channel number in to [15:12] bit
            if (addr == CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS||
                addr == CAEN_DGTZ_CHANNEL_THRESHOLD_ADD)
                    val32 |= ((ch&0xF)<<12);
            
            //0x1n98, add channel number in to [19:16] bit
            if (addr == CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS ||
                addr == CAEN_DGTZ_CHANNEL_DAC_ADD)
                    val32 |= ((ch&0xF)<<16);

            //mask group, for channel based reg 0x10xx
            if ( (addr&0xF000) != 0x8000){
                addr |= ((gr&0x3)<<8);
                snprintf(sres, SRES_MAX_LEN, "%s:%d.%d", sres, gr, ch); //add 2 log
            }

            snprintf(sres, SRES_MAX_LEN, "reg wr %s %04X", ra.name, addr);

            ret_err = DEVICE_REG_WRITE32(caenh, addr, val32 );
            
                if (ret_err==CAENComm_Success){
                        PRN_DIAG(" wr %s,%04x=%04x\n", name, addr, val32);

                        snprintf(sres, SRES_MAX_LEN, "%s=0x%x", sres, val32);
                        return drv_err_ok;
                }else{ //comm_read error
                    snprintf(sres, SRES_MAX_LEN, "%s err=%s", sres, caen_err2str(ret_err));
                    return ret_err; //caen_comm_read error
                }
            }
    }
    //string name for register dndt exist
    snprintf(sres,SRES_MAX_LEN, "reg wr %s err=%s", name, caen_err2str(drv_err_not_found));
    return drv_err_not_found;  //register name not found
}


//------------------------------------------------------------------------------
//TEST

//simple read register
uint32_t caen_reg_read(uint32_t addr, const char* msg){
    uint32_t data32;
    int err = CAENComm_Read32(caenh, addr, &data32);
    printf ("reg %04x rd %s %x %s", addr, caen_err2str(err), data32, msg);
    return data32;
}

//simple write register
void caen_reg_wrte(uint32_t addr, uint32_t data32, const char* msg){
    int err = CAENComm_Write32(caenh, addr, data32);
    printf ("reg %04x wr %s %x %s", addr, caen_err2str(err), data32, msg);
}

//simple stat
int caen_acq_stat(){
    uint32_t data32;
    int ret = CAENComm_Read32(caenh, CAEN_DGTZ_ACQ_STATUS_ADD, &data32);
    if (ret)  return -1;
    uint8_t s=3; //2=run, 3=data_ready, 4=full
    return (data32&(1<<s))>>s;
}

//msec delay
#define MY_CPU_FREQ 3000 //single core turbo max freq, xeon E5 2678V3
inline void dumb_sleep(const uint32_t usec){
    for ( uint32_t s = usec*MY_CPU_FREQ; s; --s ) __asm__("nop");
}

//delta time
static uint32_t dump_dt(struct _timeb tb1, struct _timeb tb2){
    return (uint32_t) (1000.0 * (tb2.time - tb1.time) + (tb2.millitm - tb1.millitm));
}

//simple BLT read, no parse
uint32_t dump_blt_read(uint32_t *buf){
    uint32_t nw;
    if ( CAENComm_BLTRead(caenh, 0, buf, 4, &nw) ==0 )
         if ( CAENComm_BLTRead(caenh, 0, buf, (buf[0]&(0xFFFFFFF))-4, &nw) ==0 )
            return nw;
    return 0;
}
    //CAENComm_ErrorCode err;
    //if ( CAENComm_MultiRead32 (caenh, 0, 1, buf, &err)==0){ //not wprks


void test_caen_speed(){
    //uint32_t i=1000; while(i){ i--; dumb_sleep(1000);} 

    if (!CAENComm_OpenDevice (CAENComm_USB, 0, 0, 0, &caenh)){
        printf("caen speed test\n");
        //caen_reg_wrte (CAEN_DGTZ_SW_RESET_ADD, 0x123, "sw_reset"); 
        caen_reg_wrte (CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, 0x1, "nim_ttl");
        caen_reg_wrte (CAEN_DGTZ_CUSTOM_SIZE_REG, 0x0, "rec_len"); //1024,520,256,136
        caen_reg_wrte (CAEN_DGTZ_DRS4_FREQUENCY_REG_WRITE, 0x0, "freq"); //5, 2.5, 1, 750
        caen_reg_wrte (CAEN_DGTZ_CH_ENABLE_ADD, 0x3, "gr_en");
        caen_reg_wrte (CAEN_DGTZ_BROAD_CH_CLEAR_CTRL_ADD, 0xFFFF, "cfg_clr");
        //caen_reg_wrte (CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, 0x110, "tr_nodig");
        caen_reg_wrte (CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, 0x910, "tr_dig");
        caen_reg_wrte (CAEN_DGTZ_TRIGGER_SRC_ENABLE_ADD, (3UL<<30), "tr_slow_sw");

          struct _timeb tb1,tb2;
          uint32_t dt,odo=0,i=0,rd=0,data32;
          uint32_t buf[7000];

          caen_reg_wrte (CAEN_DGTZ_ACQ_CONTROL_ADD,(1<<2),"\nack_start");
            //caen_reg_wrte (CAEN_DGTZ_SW_TRIGGER_ADD,1,"sw_trg");

            uint8_t run=1;
            _ftime(&tb1);
            while (run){                
                if (caen_acq_stat(0))
                    if (rd+=dump_blt_read(buf)) 
                        odo++;
                if (++i>3000) run=0;
            }
            _ftime(&tb2);
          caen_reg_wrte (CAEN_DGTZ_ACQ_CONTROL_ADD,0,"ack_stop");
          if (caen_acq_stat(0)) dump_blt_read(buf);
        
        
        dt = dump_dt(tb1,tb2);
        printf("\ndt=%dms, odo=%d, speed=%dHZ, read=%dMB, speed=%.1fMBs\n",
         dt, odo-1, 1000*odo/dt, rd*4/1000000, (float)rd*4/(dt*1000.0) );

        CAENComm_CloseDevice(caenh);
    } else printf("open_err\n");
}

void test_caen2(){
    int caenh, ret; 
    ret = CAENComm_OpenDevice (CAENComm_USB, 0, 0, 0, &caenh);
    printf("opn %d, ",ret);
    ret = CAENComm_Write32(caenh, 0x8120, 0x1);
    printf("set %d, ",ret);
    ret = CAENComm_CloseDevice(caenh);
    printf("clz %d.",ret);
}


//EOF

//------------------------------------------------------------------------------
/* O L D */
/*

void test_caen1(){
    uint8_t ret; 
    char sval[16], sres[16]; uint32_t ival;
    sval[0]=0;

    ret = caen_board_loc_open(0,0,sres); PRN_RESULT;
    
    ret = caen_reg_write_name("agr_ch_dc",  0,3,87,   sval,sres);   PRN_RESULT;
    //ret = caen_reg_write_name("gr_ch_dc",   0,0xF,1,   sval,sres);   PRN_RESULT;
    ret = caen_reg_read_name ("gr_ch_dc",   0,1,&ival,sval,sres);   PRN_RESULT;

//ret = caen_reg_write_name("post_tr",0,3,12,sval,sres);            PRN_RESULT;
    
    ret = caen_reg_write_name("dummy",1,123,sres);          PRN_RESULT;
    ret = caen_reg_read_name ("dummy",1,&ival,sval,sres);   PRN_RESULT;

    ret = caen_reg_write_name ("tr_polarity",1,0,sres);   PRN_RESULT;
    ret = caen_reg_read_name  ("tr_polarity",1,&ival,sval,sres);   PRN_RESULT;

    ret = caen_reg_write_name("sel_gr",4,0,sres);        PRN_RESULT;
    ret = caen_reg_read_name ("sel_gr",5,&ival,sval,sres); PRN_RESULT;

    ret = caen_reg_write_name("drs4_freq",4,123,sres);        PRN_RESULT;
    ret = caen_reg_read_name ("drs4_freq",5,&ival,sval,sres); PRN_RESULT;

    ret = caen_reg_write_name("post_tr",0,3,sres);            PRN_RESULT;
    ret = caen_reg_read_name ("post_tr",1,&ival,sval,sres);   PRN_RESULT;

    ret = caen_reg_read_name("ver_amc",0,&ival,sval,sres);   PRN_RESULT;
    ret = caen_reg_read_name("ver_roc",0,&ival,sval,sres);   PRN_RESULT;
    ret = caen_reg_read_name("ver_mem",0,0,&ival,sval,sres);   PRN_RESULT;
    ret = caen_reg_read_name("drs4_temp",0,0,&ival,sval,sres); PRN_RESULT;

    ret = caen_board_loc_close(sres); PRN_RESULT;
}

//write register with check
void caen_reg_wrte_chk(uint32_t addr, uint32_t data32w, char* msg){
    int ret;
    uint32_t data32r;
    printf ("write %s", msg);
    ret = CAENComm_Read32(caenh, addr, &data32r);
    if (ret) printf (" rerr %d", ret);
    if (data32r != data32w){
        ret = CAENComm_Write32(caenh, addr, data32w );
        if (ret) printf (" wr_err %d", ret);
            else printf (" wr_ok");
        ret = CAENComm_Read32 (caenh, addr, &data32r);
        if (ret) printf (" chk_rerr %d\n", ret);
            else
                if (data32r == data32w) printf (" chk_ok\n" );
                                   else printf (" chk_nok %d\n",data32r);
        } 
        else printf (" yet\n");
}

//0x1088 group status
void caen_grp_stat(int gr){
    int ret;
    uint32_t data32;
    ret = CAENComm_Read32(caenh, CAEN_DGTZ_CHANNEL_STATUS_BASE_ADDRESS|((gr&0x3)<<8), &data32);
    if (ret) sprintf (diag_msg, "gr_stat %d",ret);
        else 
    sprintf (diag_msg, "gr%d_status %s%s%s%s%s%s\n", gr,
    data32&(1<<0)?"mem_full ":"",
    data32&(1<<1)?"mem_empty ":"",
    data32&(1<<2)?"spi_bisy ":"",
    data32&(1<<6)?"pll_even_lock ":"",
    data32&(1<<7)?"pll_odd_lock ":"",
    data32&(1<<8)?"drs4_bisy ":""
    );
    push_msg();
}

//0x1088 check is spi bisy, 0=ok
int caen_grp_stat_chk_spi(int gr){
    uint32_t data32;
    int ret= CAENComm_Read32(caenh, CAEN_DGTZ_CHANNEL_STATUS_BASE_ADDRESS|((gr&0x3)<<8), &data32);
    if (ret) {sprintf (diag_msg, "chk_spi_err %d",ret); push_msg(); return -1;}
    return (data32&(1<<2));
}

//return: buf_data_ready, buffer_full, is_run
int caen_acq_stat(int wh){
    int ret;
    uint32_t data32;
    ret = CAENComm_Read32(caenh, CAEN_DGTZ_ACQ_STATUS_ADD, &data32);
    if (ret) {sprintf (diag_msg, "acq_err %d",ret); push_msg(); return -1;}

    switch(wh){
    case 0: ret = (data32&(1<<3))>>3; break;
    case 1: ret = (data32&(1<<4))>>4; break;
    case 2: ret = (data32&(1<<2))>>2; break;
    default: break;
    }
    return ret;
}

//output: silent, short_inline, full
int caen_acq_stat_full(int mode){
    int ret;
    uint32_t data32;
    ret = CAENComm_Read32(caenh, CAEN_DGTZ_ACQ_STATUS_ADD, &data32);
    if (ret) {sprintf (diag_msg, "acq_err %d",ret); return -1;}
    switch (mode) {
    case 0: break;
    case 1:
        sprintf (diag_msg,"-%s%s%s%s%s- ",
        data32&(1<<2)?"r":"s",  //run/stop
        data32&(1<<3)?"a":"e",  //evt_buff avali/empty
        data32&(1<<4)?"f":"",   //evt_buff full
        data32&(1<<7)?"" :"p",  //pll err
        data32&(1<<8)?"" :"b"); //board err
        break;
    case 2:
        sprintf (diag_msg,"acq_status %s%s%s%s%s\n",
        data32&(1<<2)?"run ":"stop ",
        data32&(1<<3)?"evt_ready ":"evt_empty ",
        data32&(1<<4)?"evt_full " :"evt_nfull ",
        data32&(1<<7)?"pll_ok ":"pll_unlock ",
        data32&(1<<8)?"brd_ready ":"brd_n_ready ");
        break;
    default: break;
    }
    push_msg();
    return data32&0xFF;
}


//sw_reset: reset all registers to defaults
void caen_board_reset(){
    caen_reg_wrte(CAEN_DGTZ_SW_RESET_ADD,0xff,"sw_reset");
}

//sw_clear (each run): clear some registers like evt_num, trg_timestemp etc
void caen_board_clear(){
    caen_reg_wrte(CAEN_DGTZ_SW_CLEAR_ADD,0xff,"sw_clear");
}

//reload = sw_reset, ROM_reload, PLL_reconfig
void caen_board_reload(){
    caen_reg_wrte(CAEN_DGTZ_RELOAD_CONFIG_ADD,0xff,"cfg_reload");
}

//0x1nA0 RDS4 temperature
int caen_conf_get_temp(int gr){
    uint32_t data32=caen_reg_read(
    0x10A0|((gr&0x3)<<8), //gr 0-3
    "drs4_temp");
    return data32&0xFF;  //16b
}

//0x8120 select group to digitize 0-F, mask, 
void caen_conf_gr(int gr){
    caen_reg_wrte(CAEN_DGTZ_CH_ENABLE_ADD,(gr&0xF),"gr_select");
}

 //digitise fast tr in to ch8 each gr
void caen_conf_tr_dig(){
    caen_reg_wrte(CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, (1<<11), "tr0_dig");
}

//do not digitise tr
void caen_conf_tr_nodig(){
    caen_reg_wrte(CAEN_DGTZ_BROAD_CH_CLEAR_CTRL_ADD,    (1<<11), "tr0_nodig");
}

//use tr0 as a fast trigger
void caen_conf_tr_use(){
    caen_reg_wrte(CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, (1<<12), "tr0_use");
}

//dont use tr0 trigger
void caen_conf_tr_nouse(){
    caen_reg_wrte(CAEN_DGTZ_BROAD_CH_CLEAR_CTRL_ADD,    (1<<12), "tr0_nouse");
}

//channel dc offset set, gr 0-1(F), ch 0-7(F), val 0-FFFF
void caen_conf_set_ch_dc(int gr, int ch, int val){
    if (!caen_grp_stat_chk_spi) return;   
    if (gr==0xF)
        caen_reg_wrte(0x8098, ((ch&0xF)<<16)+(val & 0xFFFF), "ch_dc_offs_gr");
    else 
        caen_reg_wrte(
        CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS|((gr&0x3)<<8), //0x1n98, 0<n<3
        ((ch&0xF)<<16)+(val & 0xFFFF), "ch_dc_offs");     //16b value + 4b channel (0xF=all)
}

//channel dc offset get, gr 0-1, ch 0-7
int caen_conf_get_ch_dc(int gr, int ch){
    caen_reg_wrte(
        0x10A4|((gr&0x3)<<8), //0x1nA4, 0<n<3
        ch&0x7,               //ch 0-7
        "sel_ch");
    uint32_t data32=caen_reg_read(
        CAEN_DGTZ_CHANNEL_DAC_BASE_ADDRESS|((gr&0x3)<<8), //gr 0-3
        "ch_dc_offs");
    return data32&0xFFFF;  //16b
}

//channel trigger threshould set, gr 0-1(F), ch 0-7(F), val 12b 0-3FFF
void caen_conf_set_ch_tr(int gr, int ch, int val){
    if (gr==0xF)
        caen_reg_wrte(0x8080,         //all groups adress
        ((ch&0xF)<<12)+(val & 0x3FF), //0-3(F)gr - ignore, 12b value
        "ch_tr_thr_gr");
    else 
        caen_reg_wrte(
        CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS|((gr&0x3)<<8), //0x1n98, 0<n<3
        ((ch&0xF)<<12)+(val & 0x3FF), "ch_tr_thr"); //12b value + 4b channel (0xF=all)
}

//channel trigger threshould get, gr 0-3, ch 0-7
int caen_conf_get_ch_tr(int gr, int ch){
    caen_reg_wrte(
        0x10A4|((gr&0x3)<<8), //0x1nA4, 0<n<3
        ch&0x7,               //ch 0-7
        "sel_ch");
    uint32_t data32=caen_reg_read(
        CAEN_DGTZ_CHANNEL_THRESHOLD_BASE_ADDRESS|((gr&0x3)<<8),
        "ch_tr_thr");
    return data32&0x3FF;      //12b
}

//tr dc threshold set, gr 0-1(F), val 16b 0-FFFF
//gr0=gr1=tr0, gr2=gr3=tr1
void caen_conf_set_tr_tr(int gr, int val){
    if (!caen_grp_stat_chk_spi) return;   
    if (gr==0xF)
        caen_reg_wrte(0x80D4, //all groups adress 
        (val&0xFFFF),         //16b value
        "tr_tr_gr");
    else 
        caen_reg_wrte(
        CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS|((gr&0x3)<<8), //0x1nD4, 0<n<3
        (val & 0xFFFF), "tr_tr"); //16b value
}

//tr dc offset get, gr 0-1
int caen_conf_get_tr_tr(int gr){
    uint32_t data32=caen_reg_read(
        CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS|((gr&0x3)<<8), //0x1nD4, 0<n<3
        "tr_tr");
    return data32&0xFFFF;  //16b
}

//tr dc offset set, gr 0-1(F), val 16b 0-FFFF
//gr0=gr1=tr0, gr2=gr3=tr1
void caen_conf_set_tr_dc(int gr, int val){
    if (!caen_grp_stat_chk_spi) return;   
    if (gr==0xF)
        caen_reg_wrte(0x80DC, //all groups adress 
        (val&0xFFFF),         //16b value
        "tr_dc_offs_gr");
    else 
        caen_reg_wrte(
        CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS|((gr&0x3)<<8), //0x1nDC, 0<n<3
        (val & 0xFFFF), "tr_dc_offs"); //16b value
}

//tr dc offset get, gr 0-1
int caen_conf_get_tr_dc(int gr){
    uint32_t data32=caen_reg_read(
        CAEN_DGTZ_GROUP_FASTTRG_DCOFFSET_V1742_BASE_ADDRESS|((gr&0x3)<<8), //0x1nDC, 0<n<3
        "tr_dc_offs");
    return data32&0xFFFF;  //16b
}

void caen_board_set(){
    //caen_board_reset();

    //switch led "NIM<->TTL"
    if  (caen_reg_read(CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD,"nim-ttl")&0x1)
         caen_reg_wrte(CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, 0x00,"swtch_nim");
    else caen_reg_wrte(CAEN_DGTZ_FRONT_PANEL_IO_CTRL_ADD, 0x01,"swtch_ttl");

    caen_conf_get_temp(0);

    //00 = 5 GS/s (default);01 = 2.5 GS/s;10 = 1 GS/s;11 = 750 MS/s; 0x1nD8
    caen_reg_wrte(CAEN_DGTZ_DRS4_FREQUENCY_REG_WRITE,2,"drs4_freq");

    //00 = 1024 samples/ch; 01 = 520 samples/ch; 10 = 256 samples/ch; 11 = 136 samples/ch.
    caen_reg_wrte(CAEN_DGTZ_CUSTOM_SIZE_REG,  0, "sampl_per_ch");

    //poet trg delay x*8.5ns+115/42ns
    caen_reg_wrte(CAEN_DGTZ_POST_TRIG_ADD, 100, "post_trg");
    
    caen_conf_gr(3); //set group mask

    //board conf: tr0 digitize, use TR0
    //caen_conf_tr_nodig();
    caen_conf_tr_dig();
    caen_conf_tr_use();
   
    //offsets ant thr
    caen_conf_set_ch_dc(0xF,0xF,0x7FFF);
    caen_conf_get_ch_dc(1,5);

    caen_conf_set_ch_tr(0,5,0x123);
    caen_conf_get_ch_tr(0,5);

    caen_conf_set_tr_tr(0,25000);
    caen_conf_get_tr_tr(0);

    caen_conf_set_tr_dc(0, 0x7FFF);
    caen_conf_get_tr_dc(0);
 
    caen_reg_wrte(0xEF1C,100,"blt_evts");
    caen_reg_read (0xEF1C,"blt_evts");
}


//0xEF04.1 - Readout Status. Indicates if there are events stored ready for readout.
int8_t caen_board_readout(){
    uint32_t val=0;
    int8_t ret = CAENComm_Read32 (caenh, CAEN_DGTZ_VME_STATUS_ADD, &val);
    if (ret == CAENComm_Success) 
             return val & 0x1;
        else return ret;
}

//NOT work at USB version
void caen_dbg_read(){
    //CAENComm_IRQEnable(caenh);
    uint32_t val;
    
    if ( !CAENComm_Read32 (caenh, CAEN_DGTZ_BROAD_CH_CTRL_ADD, &val)){
        printf (" bc=%x",val);
        CAENComm_Read32 (caenh, 0x10DC, &val);
        printf (", trdc=%x",val);
        CAENComm_Read32 (caenh, 0x10D4, &val);
        printf (", trtr=%x",val);
    }

}

//device versions
struct caen_ver{
    char amc_ver[16];
    char roc_ver[16];
    char brd_ver[16];
    char sw_ver [16];
};
struct caen_ver cvt;

//AMC, ROC, board, sw
void caen_board_info(){
    int ret;
    uint32_t data32;

    ret = CAENComm_Read32(caenh,CAEN_DGTZ_CHANNEL_AMC_FPGA_FW_BASE_ADDRESS,&data32); //0x1n8C
    if (ret) {sprintf (diag_msg, "amc_err %d",ret); push_msg();}
        else {sprintf (cvt.amc_ver, "%d.%02d %02x.%02d.%02d",
                        (data32>>8 &0xFF),
                        (data32>>0 &0xFF),
                        (data32>>16&0xFF),
                        (data32>>24&0xF ),
                        (data32>>28&0xF )
                        );
              sprintf (diag_msg, "AMC_ver %s\n", cvt.amc_ver); push_msg();
              }

    ret = CAENComm_Read32(caenh,CAEN_DGTZ_FW_REV_ADD,&data32);  //0x8124
    if (ret) {sprintf (diag_msg, "roc_err %d",ret); push_msg();}
        else {sprintf (cvt.amc_ver, "%d.%02d %02x.%02d.%02d",
                        (data32>>8 &0xFF),
                        (data32>>0 &0xFF),
                        (data32>>16&0xFF),
                        (data32>>24&0xF ),
                        (data32>>28&0xF )
                        );
              sprintf (diag_msg, "ROC_ver %s\n", cvt.amc_ver); push_msg();
              }

    ret = CAENComm_Read32(caenh,CAEN_DGTZ_BOARD_INFO_ADD,&data32); //0x8140
    if (ret) {sprintf (diag_msg, "board_info_err %d",ret); push_msg();}
        else {sprintf (cvt.brd_ver, "%s_family %s_evts %s_gr",
                        ((data32>>0 &0xFF) == 6)?"742":"unknwn",
                        ((data32>>8 &0xFF) == 1)?"128":"1024",
                        ((data32>>16&0xFF) == 2)?"2":"4"
                        );
              sprintf (diag_msg, "board_info %s\n", cvt.brd_ver); push_msg();
              }

    char cdata [32];
    ret = CAENComm_SWRelease(cdata);
    if (ret) {sprintf (diag_msg, "sw_rel_err %d",ret); push_msg();}
        else {sprintf (cvt.sw_ver, "%s",cdata);
              sprintf (diag_msg, "sw_rel %s\n", cvt.sw_ver); push_msg();
        }
}

    if (ret) {sprintf (diag_msg, "acq_err %d",ret); return -1;}
    switch (mode) {
    case 0: break;
    case 1:
        sprintf (diag_msg,"-%s%s%s%s%s- ",
        data32&(1<<2)?"r":"s",  //run/stop
        data32&(1<<3)?"a":"e",  //evt_buff avali/empty
        data32&(1<<4)?"f":"",   //evt_buff full
        data32&(1<<7)?"" :"p",  //pll err
        data32&(1<<8)?"" :"b"); //board err
        break;
    case 2:
        sprintf (diag_msg,"acq_status %s%s%s%s%s\n",
        data32&(1<<2)?"run ":"stop ",
        data32&(1<<3)?"evt_ready ":"evt_empty ",
        data32&(1<<4)?"evt_full " :"evt_nfull ",
        data32&(1<<7)?"pll_ok ":"pll_unlock ",
        data32&(1<<8)?"brd_ready ":"brd_n_ready ");
        break;
    default: break;
    }
    push_msg();
    return data32&0xFF;



    int caen_ret_err;
char diag_msg[64];
void push_msg() {printf("%s\n",diag_msg);}
void wrtie_data();

//caen dt7542 digitized data buffer
//channels: gr0.8ch + 1tr0 + gr1.8ch + 1tr0, 18 samples 16bit each (12 bit meanings)
//samples: 1024 samples for each 18 channel, 
//zero filled if no digitized, see ch0-1 and tr0 settings 
#define CN_SN 1024  //caen samples num 1024
#define CN_CN 18    //caen total channels num 2 group, 8+1ch each gr
#define CN_GS 13832 //caen group shift 12288 group_data +1536 tr_data + 4gr_tt +4 gr_desc 
#define CN_SC 18432 //caen total 16b samples 1024* ((8ch+1tr)*2gr) in buffer
uint16_t digibuf[CN_SC];

void caen_btl_parse(){
    //parse header
    char diag_bmsg[64];
    sprintf(diag_bmsg,"btl ");
    int ret;
    uint32_t hdr[4];
    memset(hdr,0,4*4);
    int nw=0;
    ret = CAENComm_BLTRead(caenh, 0, hdr, 4, &nw);
    if (ret) {sprintf(diag_msg,"hdr_rd %s\n", caen_err2str(ret)); push_msg(); return; }
        else 
            sprintf (diag_bmsg+strlen(diag_bmsg), "hdr %ssz%d %sg%d e%03d ett%s%d, ",
            hdr[0]&(0x0A<<28)?"":"hnok ", //0101
            hdr[0]&(0xFFFFFFF),           //evt size
            hdr[1]&(0x01<<26)?"BF ":"",   //brd fail
            hdr[1]&(0x03),                //grp mask
            hdr[2]&(0x7FFFFF),            //evt counter
            hdr[3]&(0x01<<31)?"OF ":"",   //ovr flow
            hdr[3]&(0x7FFFFFFF)           //evt time tag
            );
    //parce evt body
    uint32_t  evtc =(hdr[0]&(0xFFFFFFF))-4; //event size in 32b, minus hdr sz
    if (!evtc){ sprintf (diag_bmsg+strlen(diag_bmsg), "zero_payload\n"); 
                strcpy(diag_msg,diag_bmsg); push_msg(); return;}
    
    uint32_t *buff = malloc(evtc*4);
    if (!buff) {sprintf (diag_bmsg+strlen(diag_bmsg),"btl_mem_err\n"); 
                strcpy(diag_msg,diag_bmsg); push_msg(); return;}
    memset(digibuf,0,CN_SC*2);

    ret = CAENComm_BLTRead(caenh, 0, buff, evtc, &nw);
    if (ret) {sprintf (diag_bmsg+strlen(diag_bmsg),"btl_read_err\n"); free(buff);
              strcpy(diag_msg,diag_bmsg); push_msg(); return;}

    sprintf (diag_bmsg+strlen(diag_bmsg), 
             "body sz%d st%04d fq%d tr%d gtt%d\n", nw, //32b words readed
        (buff[0] & 0x3FF00000) >>20,   //domino start cell 0..1024
        (buff[0] & 0x30000)    >>16,   //freq
        (buff[0] & 0x1000)     >>12,   //TR on
        //(buff[0] & 0xFFF)      >>0,  //sz 12*1024*8/32= 0xc00 alws
        (buff[nw-1] & 0x3FFFFFFF)>>0); //group tt
    //buff8[0]=3; buff8[1]=0x18; buff8[2]=0x80; buff8[3]=0x0; //test data    

    uint8_t  gm = hdr [1]& 0x03;         //group mask 00,01,10,11=both
    uint8_t  tm = (buff[0]& 0x1000)>>12; //tr0 trigger mask 0,1
    uint16_t lb,rb,trv;  //left right data from 12+12=24=8*3, trig value
    uint32_t as;         //address of 8bit source buffer
    uint32_t ad;         //address of 16bit dest buffer
    uint8_t *buff8;         //pointer to 8bit data representation
    buff8 = (char*) buff+4; //shift gr desc header
    

    for (uint16_t i=0; i<CN_SN; i++){  //1024 points

        for (uint16_t k=0; k<4; k++){ //3*32b=8b*3 *4
            if (gm){  //gr0 OR gr1 OR both, gr mask
                as = i*12+k*3;
                lb= (buff8[as]) + ((buff8[as+1]&0x0F)<<8);
                rb=((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4);
                ad = i*CN_CN + k*2; //this is 1st gr data block 
                ad += (gm==2)?9:0;  //OR 2nd if only 2nd group mask
                digibuf[ad]  =lb; 
                digibuf[ad+1]=rb;
            }
            if (gm==3){  //gr0 AND gr1, gr mask
                as = i*12+k*3 + CN_GS;
                lb= (buff8[as]) + ((buff8[as+1]&0x0F)<<8);
                rb=((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4);
                ad = i*CN_CN+k*2 +9;
                digibuf[ad]=lb;
                digibuf[ad+1]=rb;
            }
        }

        if (tm){ //if TR digitzd

            if (gm){  //gr0 OR gr1 OR both, gr mask
            as=12*1024+(i>>1)*3;   //TR data at the tail
                if (!i%2) trv = (buff8[as]) + ((buff8[as+1]&0x0F)<<8);
                    else  trv =((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4);
            ad = i*CN_CN+8;     //1st gr
            ad += (gm==2)?9:0;  //OR 2nd if only 2nd group mask
            digibuf[ad]=trv;    //9th channel is trigger of first drs4
            }

            if (gm==3){
            as=12*1024+(i>>1)*3 + CN_GS;   //TR data at the tail
                if (!i%2) trv = (buff8[as]) + ((buff8[as+1]&0x0F)<<8);
                    else  trv =((buff8[as+1]&0xF0)>>4) + (buff8[as+2]<<4);
            ad = i*CN_CN+8+9;    
            digibuf[ad]=trv;   //18th channel is trigger of second drs4
            }
        }
    }
    free(buff);
    strcpy(diag_msg,diag_bmsg);
    push_msg();
    wrtie_data(digibuf);
}

void wrtie_data(uint16_t *buf){
    //print to file
    time_t t = time(NULL);
    struct tm lt  = *localtime(&t);

    FILE *fptr;
    char fname[32];
    sprintf(fname,"csv/caen-%03d%02d%02d%02d.csv", lt.tm_yday, lt.tm_hour, lt.tm_min, lt.tm_sec);
    fptr = fopen(fname,"w");
    printf("file %s", fname);
    if (fptr) {
        for (uint16_t i=0; i<1024; i++){  //1024 points
            for (uint16_t k=0; k<18; k++)  //8+1+8+1 ch
                fprintf(fptr,"%04d ", buf[i*18+k]);
            fprintf(fptr,"\n");
        }
    fclose(fptr);
    printf(" ok\n");
    } else printf(" err\n");
}

*/
