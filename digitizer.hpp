/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * digitizer.hpp
 * Caen digitizer logic */

#pragma once

#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cstring>
#include <math.h>
#include <regex>
#include <random>
#include <iomanip>

using std::cout;
using std::thread;
using std::string;
using std::vector;
using std::array;
using std::unordered_map;
using std::regex;

#include "pugixml.hpp"

#include "mvp.hpp"
#include "digi-gui.hpp"
#include "ts-calc.hpp"

namespace app_const{
    const string def_con_view_name   = "vcon";  //View console
    const string def_con_err_marker  = "err";   //colorer if in log
    
    const string def_glob_signal_cmd  = "CMD";  //View name for internal command

    //const string def_con   = "";
    enum internal_cmd{
        cmd_halt = 1,  //stop threads
        cmd_kick = 2,  //kick sw trigger
        cmd_rota = 3   //wheel rotated
    };
}


class ConLogger: public DigiView{
public:
    ConLogger():DigiView(app_const::def_con_view_name){
    }

    void Init();
    void Config(const string &param, const int val1, const int val2){}//stub

    void MainLoop();
    
    void CallView(const string &param, const int val);
    void CallView(const string &param, const string &val);
    void CallView(const string &param, const uint16_t *val);

    void PushLog(const std::string& txt);

private:
    const string err_marker = app_const::def_con_err_marker; //Error marker inside string
    void* hConsole;             //OS console header
    void ConColorSet();         //set color console mode for Win10
    void PushLogLine(const std::string& line); //push line to buf
};

extern ConLogger *conlog;

//ein setting
struct ein_set{
    ein_set(string name_, string widget_, int ival_=0):
        name(name_),widget(widget_),xml(false){
            SetVal(0);
            }

    void SetVal(const int ival_, const bool xml_=false){
        ival=ival_;
        sval=std::to_string(ival);
        xml=xml_;
        }

    void SetVal(const char* sval_, const bool xml_=false){
        sval=string(sval_);
        ival=atoi(sval_);
        xml=xml_;
        }

    string name;   //name for map, dublicat
    string widget; //corresponding widget name
    string sval;   //string value
    int    ival;   //integer value
    bool   xml;    //got from XML file and will save
};

//setting names
namespace set_5742{
    const string drs4_freq  = "drs4_freq";
    const string rec_len    = "rec_len";
    const string tr_delay   = "tr_delay";
    const string tr_polar   = "tr_polar";
    const string evt_blt    = "evt_blt";
    const string io_lev     = "io_lev";
    const string drs4_temp  = "drs4_temp";

    const string ver_drv    = "ver_drv";
    const string ver_roc    = "ver_roc";
    const string ver_amc    = "ver_amc";
    const string ver_mem    = "ver_mem";
    
    const string tr_fast    = "tr_fast";
    const string tr_slow    = "tr_slow" ;
    const string tr_soft    = "tr_soft";
    const string tr_soft_kick  = "tr_soft_kick";

    const string tr_self    = "tr_self";

    const string run        = "acq_run";
    const string test       = "mode_test";    
    const string gpo        = "force_gpo";    
    const string reset      = "sw_reset";    

    const string xml_root    ="digitizer";
    const string gr_tag      = "gr";
    const string ch_tag      = "ch";
    const string ch_mask     = "mask";
    const string ch_view     = "view";
    const string ch_dc       = "dc";
    const string ch_tr       = "tr";
    const uint8_t ch_all     = 0xFF;
    const int tag_empty      = 0xFFFF;

//    const string     = "";    
}

//widget names
namespace wid_5742{
    const string drs4_freq  = "ParFreq";
    const string rec_len    = "ParRecord";
    const string tr_delay   = "ParTrDelay";
    const string tr_polar   = "ParTrPolar";
    const string evt_blt    = "ParEvtBlt";
    const string io_lev     = "ParLemo";
    const string drs4_temp  = "ParTemp";
    const string ver_drv    = "ParBoard";
    const string ver_roc    = "ParROC";
    const string ver_amc    = "ParAMC";
    const string ver_mem    = "ParBoard2";
    
    const string coax_mark      = "BtnCoax";
    const string coax_ch_mark   = "BtnCoaxCh";
    const string coax_gr_mark   = "BtnCoaxGr";
    const string coax_tr_mark   = "BtnCoaxTr";
    const string coax_mode_mark = "BtnCoaxMode";
    const string tab_view   = "BtnCoaxModeView";
    const string tab_mask   = "BtnCoaxModeMask";
    const string tab_dc     = "BtnCoaxModeDc";
    const string tab_tr     = "BtnCoaxModeTr";

    const string tr_fast    = "BtnStolb11";
    const string tr_slow    = "BtnStolb12";
    const string tr_soft    = "BtnStolb13";
    const string tr_self    = "BtnStolb14";

    const string tr_soft_kick = "BtnStolb21";
    const string tr_soft_dis  = "BtnStolb22a"; //overlap
    const string tr_soft_off  = "BtnStolb22b"; 
    const string tr_soft_cont = "BtnStolb23";
    const string tr_stolb2_mask = "BtnStolb2";

    const string but_file_save  = "BtnFileSave";
    const string but_board_open = "BtnOpen";
    const string but_rt_rec     = "BtnRtRec";
    const string but_run        = "BtnRun";
    const string but_gpo        = "BtnRunGPO";
    const string but_res        = "BtnRunRes";
    const string but_test       = "BtnRunTest";

    const string led_open       = "LedBOpen";
    const string led_run        = "LedYRun";
    const string led_trg        = "LedRTrg";
    const string led_rdy        = "LedGRdy";
    const string led_busy       = "LedRBusy";
    const string led_pll        = "LedGPLL";

    const string cur_chdc       = "CurChDc";
    const string cur_chth       = "CurChTh";
    const string cur_trdc       = "CurTrDc";
    const string cur_trth       = "CurTrTh";
    
    const string sig_ch         = "SigCh";
    const string sig_tr         = "SigTr";
    const string cfg_sig        = "CfgSig";

    const string calc_N         = "ArcGaugeN";
    const string calc_T         = "ArcGaugeT";

    const string rota_wheel     = "RotaWheel";
}

//ein channel set
//0xFF means not in settings
struct ch_set{
    ch_set( const int gr_=-1, 
            const int ch_=-1):
        gr  (gr_),
        ch  (ch_),
        dc  (set_5742::tag_empty),
        tr  (set_5742::tag_empty),
        mask(set_5742::tag_empty),
        view(set_5742::tag_empty),
        xml(false){}

    //ch_set(const uint8_t gr_, const uint8_t ch_):gr(gr_),ch(ch_){}
    
    inline bool operator==(const ch_set& rhs) { 
        return ( (this->gr == rhs.gr) && (this->ch == rhs.ch) );
     }

    inline bool operator!=(const ch_set& rhs) { return !(*this == rhs); }

    bool     xml;   //save to file
    uint8_t  gr;    //group or 0xFF for ALL gr (all device)
    uint8_t  ch;    //channel or 0xFF for ALL ch in gr
    uint16_t dc;    //dc bias offset
    uint16_t tr;    //trigger threshould
    uint16_t mask;  //digitize or not
    uint16_t view;  //view or not
};

class DigiSet{
public:
    DigiSet();

    void LoadXML(const string &filename);
    void SaveXML(const string &filename);
    void SaveXML();

    int link_type;
    int link_num;
    int gr_num;  //dgtzr groups count, def=2
    int ch_num;  //dgtzr channel pre group count, def=8

    std::map<string,ein_set> map_set; //separate setting map
    std::vector<ch_set>      vec_set; //cnahhel and trigger  settings

private:
    string filename; //xml config filename

    string digitizer_name; //name="dt5742"

    //BAD bind setting to gui widget
    void  insertSet(const string &set_name, const string &set_wid){
        map_set.insert({set_name, ein_set(set_name, set_wid)});
    }

    //find gr_ch set in vector
    bool findChset(const int gr, const int ch, ch_set *set){
    auto it =  std::find(vec_set.begin(), vec_set.end(), ch_set(gr, ch));
        if (it != vec_set.end()) 
             {set=&(*it); return true;}
        else {set=0;      return false;}
    }

    //find gr_ch set in vector and update
    bool updateChset(const int gr, const int ch, ch_set &set){
    auto it =  std::find(vec_set.begin(), vec_set.end(), ch_set(gr, ch));
        if (it != vec_set.end()){
            set.ch = ch;
            set.gr = gr;
            if (set.view!=set_5742::tag_empty) it->view=set.view;
            if (set.mask!=set_5742::tag_empty) it->mask=set.mask;
            if (set.dc  !=set_5742::tag_empty) it->dc=set.dc    ;
            if (set.tr  !=set_5742::tag_empty) it->tr=set.tr    ;
                                               it->xml=set.xml  ;
        return true;
        }
        else return false;
    }

    //XML parcer
    pugi::xml_document xml_doc;
    pugi::xml_node     xml_root;
    void   parse_tags_title();
    int    parse_s2int    (const string &sval);
    int    parse_intertag (const string &tag);
    int    parse_attrib   (const string &attr);
    int    parse_both     (const string &attr);
    ch_set parse_chset    ();
    ch_set parse_chset    (const string &tag);
    ch_set parse_chset    (const string &tag, const string &tag2);
    void   parse_grtr     ();
    void   printAll();   //2 debug

    void modify_intertag (const string &tag, const string &val);
    void modify_attrib   (const string &tag, const string &attr, const string &val);
};

//Digitizer device interface
class DigiDevice{
public:
    DigiDevice():is_open(0){}

    virtual int SettingRead (const string &name, const string &val)=0; //read single setting
    virtual int SettingWrite(const string &name, const string &val)=0; //write single setting

    virtual int Command     (const string &name, const string &val)=0; //send single command
    
    template<class T>
    T GetState (); //get specific device state

    virtual void BoardOpen  ()=0; //open device, sync
    virtual void BoardClose ()=0; //close device
    virtual void BoardTest  ()=0; //test if device alive
            int BoardOpenState(){return is_open;} //is board open

    string GetLastErr(){return lastErr;} //return last error
protected:
    void setBoardOpenState(int cause) {is_open=cause;} //connection status {Err<0, 0=closed, 1=open_ok}
    virtual void MergeSettings()=0; //mergre settings gottn from device and from XML setting file
    
private:
    string lastErr; //last error
    int is_open;    //is connecion opened, 0=ok
};

struct state5742{
    bool open; //board conn open
    bool run;  //device run
    bool trg;  //triggered
    bool rdy;  //ready fo send
    bool busy; //buffer full
    bool pll;  //pll dead
};

class Digi5742: public DigiDevice{
public:
    void SettingsReadAll ();
    void SettingsWriteAll();

    int SettingRead (const string &name, const string &val);
    int SettingWrite(const string &name, const string &val);
    void MergeSettings();
    
    int Command     (const string &name, const string &val);
    
    void BoardOpen (); //open board by comm channel USB
    void BoardClose(); //close board conn
    void BoardTest (); //test of connection is alive

    DigiSet   dev_sets; //settings from device
private:
};

//Global logger 
class Digitizer;
extern Digitizer *digiglob; //global 
void PushLog(const std::string& line);

//Main digitizer class
class Digitizer: public DigiModel{
public:
    Digitizer(){
        digiglob=this;
        sets = &digi.dev_sets;
        }

    void LoadConf(const std::string& fname){
        conf_fname=fname;
        ag.SetWindowTitle(conf_fname);
        digi.dev_sets.LoadXML (conf_fname);
    }

    void Run(){ 
        PushView(&cl);
        PushView(&ag);
        LoopStart();    // all loops =>
        LoopWait();
        }

    Digi5742 digi; //Digitizer device
    AllGui     ag; //GUI View   
    ConLogger  cl; //Console View
    
    void DataIRQ(const caen5742_evt *evt);  //data get board callback
    void DataCalc(const caen5742_evt &evt); //calculate result

private:
    void DeviceOpenToggle(); //open board connection
    void DeviceClose(); //close device at close app
    void DeviceMainLoop();   //bulk data transfer

    string conf_fname;  //conf file name
};

