/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * digitizer.cpp
 * Caen digitizer logic */

#include "digitizer.hpp"

#include <windows.h>

Digitizer *digiglob; //global 

void PushLog(const std::string& line){
    if (digiglob) digiglob->DigiModel::PushLog(line);
}

void ConLogger::Init(){ConColorSet();}

void ConLogger::MainLoop(){
    while(!stop);
}

void ConLogger::CallView(const string &param, const int val){
    cout<<"conView: "<<param<<"-"<<val<<std::endl;
}

void ConLogger::CallView(const string &param, const string &val){
    cout<<"conView: "<<param<<"-"<<val<<std::endl;
}

void ConLogger::CallView(const string &param, const uint16_t *val){
    cout<<"conView: "<<param<<"-"<<val<<std::endl;
}


void ConLogger::PushLog(const std::string& txt){PushLogLine(txt);}

void ConLogger::ConColorSet(){
#ifdef _WIN32
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hConsole, &dwMode);
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hConsole, dwMode);
#elif 
#endif
}

void ConLogger::PushLogLine(const std::string& line){
    std::time_t t =  std::time(0); //insert time
    std::tm   now = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&now, "%H:%M:%S");
    auto str = oss.str();

    if (line.find(err_marker) != std::string::npos) //Error colorer
        cout<<"\033[31m"<< oss.str()+" "+line << "\033[0m" <<std::endl;
    else
        cout<< oss.str()+" "+line<<std::endl;
}

//------------------------------------------------------------------------------
//XML settings parser

//fill map strucutre and bind with GUI
//BAD - binding shouldnbe here
DigiSet::DigiSet():
    gr_num(CAEN_DT7542_GR_NUM),ch_num(CAEN_DT7542_CH_NUM), //updated after XML read
    link_type(0),link_num(0){

    //map_set.reserve(20);
    insertSet(set_5742::ver_drv,    wid_5742::ver_drv  );
    insertSet(set_5742::ver_roc,    wid_5742::ver_roc  );
    insertSet(set_5742::ver_amc,    wid_5742::ver_amc  );
    insertSet(set_5742::ver_mem,    wid_5742::ver_mem  );

    insertSet(set_5742::drs4_freq,  wid_5742::drs4_freq);
    insertSet(set_5742::rec_len,    wid_5742::rec_len  );
    insertSet(set_5742::tr_delay,   wid_5742::tr_delay );
    insertSet(set_5742::tr_polar,   wid_5742::tr_polar );
    insertSet(set_5742::evt_blt,    wid_5742::evt_blt  );
    insertSet(set_5742::io_lev,     wid_5742::io_lev   );
    insertSet(set_5742::drs4_temp,  wid_5742::drs4_temp);
   
    insertSet(set_5742::tr_slow,    wid_5742::tr_slow  );
    insertSet(set_5742::tr_fast,    wid_5742::tr_fast  );
    insertSet(set_5742::tr_soft,    wid_5742::tr_soft  );
    insertSet(set_5742::tr_self,    wid_5742::tr_self  );
    insertSet(set_5742::tr_soft_kick,  wid_5742::tr_soft_kick);

    insertSet(set_5742::run,        wid_5742::but_run  );
    insertSet(set_5742::gpo,        wid_5742::but_gpo  );
    insertSet(set_5742::reset,      wid_5742::but_res  );
    insertSet(set_5742::test,       wid_5742::but_test );

    vec_set.reserve(gr_num*(ch_num+1)+1); //(ch+all)+all
    vec_set.push_back(ch_set(set_5742::ch_all, set_5742::ch_all)); //all device
    for (int g=0; g<gr_num;g++){
        vec_set.push_back(ch_set(g, set_5742::ch_all)); //all ch in gr
        for (int c=0; c<ch_num; c++)
            vec_set.push_back(ch_set(g,c));
    }
}

//Parsers---
//check digitizer type, set groups, set link
void DigiSet::parse_tags_title(){
    if (!xml_root.child("title").attribute("name").empty()){
        digitizer_name =  xml_root.child("title").attribute("name").value();
        if (!digitizer_name.compare("dt5742")) {gr_num=2; ch_num=8;}
        if (!digitizer_name.compare("n6742"))  {gr_num=2; ch_num=8;}
        if (!digitizer_name.compare("v1742"))  {gr_num=4; ch_num=8;}
        if (!digitizer_name.compare("vx1742")) {gr_num=4; ch_num=8;}
    }

    if (!xml_root.child("connect").attribute("linktype").empty())
        link_type = xml_root.child("connect").attribute("linktype").as_int();
    if (!xml_root.child("connect").attribute("linknum").empty())        
        link_num  = xml_root.child("connect").attribute("linknum" ).as_int();
}

//string to int, include "on" "off" keywords
int DigiSet::parse_s2int (const string &sval){
    int ival = atoi(sval.c_str());
    if (!sval.compare("on"))  ival=1;
    if (!sval.compare("off")) ival=0;
    return ival;
}

//convert inter-tag value to int 
//<drs4_freq> 0 </drs4_freq>
int DigiSet::parse_intertag (const string &tag){
    if (!xml_root.child(tag.c_str()).text().empty()){
        string sval = xml_root.child(tag.c_str()).text().as_string();
        sval = std::regex_replace(sval, std::regex("^ +| +$|( ) +"), "$1"); //kill space
        return parse_s2int(sval);
        }
    return set_5742::tag_empty;
}

//convert attributes value to int 
//<gr0 dc="94" mask="off" view="on">
int DigiSet::parse_attrib (const string &attr){
    if (xml_root){
        pugi::xml_attribute av = xml_root.attribute(attr.c_str());
        if (!av.empty())
            return parse_s2int(av.as_string());
    }
    return set_5742::tag_empty;
}

//parce both, intertag overide attribute
int DigiSet::parse_both (const string &attr){
        int t=parse_intertag(attr);
        int a=parse_attrib  (attr);
        return (t==set_5742::tag_empty)?a:t;
}

//parce whole channel parameters structure
ch_set DigiSet::parse_chset (){
    ch_set ret;
    ret.dc  =parse_both(set_5742::ch_dc  );
    ret.tr  =parse_both(set_5742::ch_tr  );
    ret.mask=parse_both(set_5742::ch_mask);
    ret.view=parse_both(set_5742::ch_view);
    if ( ret.dc  !=set_5742::tag_empty ||
         ret.tr  !=set_5742::tag_empty ||
         ret.mask!=set_5742::tag_empty ||
         ret.view!=set_5742::tag_empty)
         ret.xml = true; //BAD - at least one parameter

    cout<< xml_root.path() << ((ret.xml)?"*":"") <<"="<< ret.dc <<","<< ret.tr
    <<","<< ret.mask <<","<< ret.view <<std::endl;
    return ret;
}

ch_set DigiSet::parse_chset (const string &tag){
    xml_root=xml_doc.child(set_5742::xml_root.c_str()).child(tag.c_str());
    if (xml_root)
         return parse_chset();
    else return ch_set();
    //root=doc.child(xml_root.c_str());
}

ch_set DigiSet::parse_chset (const string &tag, const string &tag2){
    xml_root=xml_doc.child(set_5742::xml_root.c_str()).child(tag.c_str()).child(tag2.c_str());
    if (xml_root)
         return parse_chset();
    else return ch_set();
}

//parce groups and trigers <gr1  dc="6" tr="92"> <tr0  dc="86" tr="7">
void DigiSet::parse_grtr(){

    //all group (whole device) settings <gr>...</gr>
    ch_set set;
    set = parse_chset (set_5742::gr_tag);
    if (set.xml)
        updateChset(set_5742::ch_all, set_5742::ch_all,set);

    //concrete group settings <gr0>...</gr0>
    string grn,chn;
    for (int g=0; g<gr_num;g++){
        grn = "gr"+std::to_string(g);
        set = parse_chset(grn);
        if (set.xml)
            updateChset(g, set_5742::ch_all,set);
        
            //channels in group settings <gr0><ch0>...</ch0></gr0>
            for (int c=0; c<ch_num; c++){
                chn = "ch"+std::to_string(c);
                //dctr_ch.push_back(parse_chset(grn,chn));
                set = parse_chset(grn,chn);
                if (set.xml)
                    updateChset(g, c, set);
            }
        }
}

void DigiSet::LoadXML(const string &fn){
    //open file
    filename=fn;
    pugi::xml_parse_result result = xml_doc.load_file(filename.c_str());
    string slog;
    if (result.status==pugi::status_ok)
         slog = "XML load ok " + filename;
    else slog = "err - " + string(result.description())+ " " + filename;
    //PushLog(slog); //no view exists yet
    cout<<slog<<std::endl; //BAD
    if (result.status!=pugi::status_ok) return;

    xml_root = xml_doc.child(set_5742::xml_root.c_str());

    //who it is
    parse_tags_title();

    //parce settings from map_set list only
    for (auto &set: map_set){
        int ret = parse_intertag(set.first);
        if (ret != set_5742::tag_empty){
            //cout<<" xml_rd "<<set.first<<"="<<set.second.sval<<std::endl;
            set.second.SetVal(ret,true);
            }
        }

    parse_grtr();

    printAll();
}

void DigiSet::printAll(){
    cout<<"Device setings:"<<std::endl;
    for (auto &set: map_set)
        cout<<set.second.name<<((set.second.xml)?"*":"")<<"="<<set.second.sval<<", ";
    cout<<std::endl;

    for (auto &set: vec_set)
        cout<<
        ((set.gr==set_5742::ch_all)?"a":std::to_string(set.gr))<<","<<
        ((set.ch==set_5742::ch_all)?"a":std::to_string(set.ch))<<
        ((set.xml)?"*":"")<<"="<<
        ((set.view==set_5742::tag_empty)?"_":std::to_string(set.view))<<","<<
        ((set.mask==set_5742::tag_empty)?"_":std::to_string(set.mask))<<","<<
        ((set.dc  ==set_5742::tag_empty)?"_":std::to_string(set.dc  ))<<","<<
        ((set.tr  ==set_5742::tag_empty)?"_":std::to_string(set.tr  ))<<"; ";
    cout<<std::endl;
}


void DigiSet::SaveXML(const string &fn){
    filename=fn;
    SaveXML();
}

void DigiSet::SaveXML(){
    pugi::xml_parse_result xml_res;
    xml_res = xml_doc.load_file(filename.c_str());
    cout  << xml_res.description() <<" xml load "<< filename << std::endl;
    if (xml_res.status!=pugi::status_ok) return;

    xml_root = xml_doc.child(set_5742::xml_root.c_str());

    modify_intertag("post_trigger2","75");
    modify_attrib  ("gr","dc","75");

    bool xml_ress = xml_doc.save_file(filename.c_str());
    if (!xml_ress) cout  << "xml_save_err "<< filename << std::endl;
}

//modify intertag value, or create new, or modify attr style to intertag
//<drs4_freq> 0 </drs4_freq>
void DigiSet::modify_intertag (const string &tag, const string &val){
    pugi::xml_node rc=xml_root.child(tag.c_str());
    if (!rc)
        rc=xml_root.append_child(tag.c_str());  //add node
    if (string(xml_root.child_value(tag.c_str())).length())
        rc.last_child().set_value(val.c_str()); //mod value
    else
        rc.append_child(pugi::node_pcdata).set_value(val.c_str()); //add value
}
void DigiSet::modify_attrib (const string &tag, const string &attr, const string &val){
    pugi::xml_node rc=xml_root.child(tag.c_str());
    if (!rc)
         rc=xml_root.append_child(tag.c_str());//add node
    if (!rc.attribute(attr.c_str()))
        rc.append_attribute(attr.c_str());     //add attr
    rc.attribute(attr.c_str()).set_value(val.c_str()); //set val
}


//------------------------------------------------------------------------------
extern "C" {
#include "caen5742.h"
}

void Digi5742::SettingsReadAll(){
    uint32_t ival;
    char sval[SVAL_MAX_LEN];
    char sres[SRES_MAX_LEN];

    //single sets
    for (auto &set: dev_sets.map_set){
        //BAD check err
        if (!caen_reg_read_name(set.first.c_str(),0,0,&ival,sval,sres)){
            PushLog(sres);

            if (set.second.xml==true && //set from xml file
                set.second.ival!=ival){              
                /*caen_reg_write_name(set.first.c_str(),0,0,set.second.ival,0,sres);
                PushLog(sres);
                caen_reg_read_name(set.first.c_str(),0,0,&ival,sval,sres);*/
                }
            //set.second.SetVal(sval); //re read end set
        }
    }
}

void Digi5742::SettingsWriteAll(){
}

// if file_set exist and not equal device, than write new set to device
// if file_set not exist - use devise settings
void Digi5742::MergeSettings(){
    SettingsReadAll();


}

int Digi5742::SettingRead (const string &name, const string &val){
    return 0;
}

int Digi5742::SettingWrite(const string &name, const string &val){
    return 0;
}

int Digi5742::Command     (const string &name, const string &val){return 0;
}


//------------------------------------------------------------------------------
//Bord opener, 2DO make remote via ip

void Digi5742::BoardOpen (){
    char sres[SRES_MAX_LEN];
    
    int ret = caen_board_loc_open(dev_sets.link_type, dev_sets.link_num,sres);
    PushLog(string(sres));
    setBoardOpenState( (ret)?ret:1 ); //0=1
    
    caen_board_reg_init(sres);
    PushLog(string(sres));
}

void Digi5742::BoardClose(){
    char sres[SRES_MAX_LEN];
    int ret = caen_board_loc_close(sres);
    setBoardOpenState( (ret)?ret:0 ); //0=0
    PushLog(string(sres));
}

void Digi5742::BoardTest (){}

void Digitizer::DeviceOpenToggle(){
    if (digi.BoardOpenState()==0){
        digi.BoardOpen();
            if (digi.BoardOpenState()==1)
                MergeReg(); //lets modele merge XML and Device settings
        }
    else{
        digi.BoardClose();
        }
    ModelDeviceOpen(digi.BoardOpenState());
}

void Digitizer::DeviceClose(){
    if (digi.BoardOpenState()==1)
        digi.BoardClose();
}

//static adapter for device callback

#include <mutex>
std::mutex mtx_sig;
Digitizer *digi_adapter_ptr = nullptr;

void DataIRQadapter(const caen5742_evt *evt){
    if (digi_adapter_ptr) {
        if (mtx_sig.try_lock()){
            digi_adapter_ptr->DataIRQ(evt); //thin out
            mtx_sig.unlock();
        }
        else cout<<"!";
    }
}

//fill data for view
void Digitizer::DataIRQ(const caen5742_evt *evt){
    UpdateSignals(evt);
    DataCalc(*evt);    
}

void Digitizer::DataCalc(const caen5742_evt& evt){
    TSalgo tsa(evt);
    prr.CallView(gui_const::def_view_name, wid_5742::calc_T, tsa.Te);
    prr.CallView(gui_const::def_view_name, wid_5742::calc_N, tsa.Ne);
}

void Digitizer::DeviceMainLoop(){
    
    uint8_t board_stop=1;  //start device thread
    digi_adapter_ptr=this;
    std::thread board_loop( caen_board_loop, &board_stop, DataIRQadapter ); //==> start device system loop

    int tc=0; //thin out
    std::chrono::steady_clock::time_point tp;
    std::chrono::high_resolution_clock::duration delay = std::chrono::milliseconds(333);

    while(stop){
        
        run_state = caen_board_status;

        //! 1ms minimum, or board craches at board_close
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); 

        //update view led
        if ( ++tc%10 && std::chrono::steady_clock::now() - tp > delay){
            tp = std::chrono::steady_clock::now();
            // cout<<"."; if (run_state.evt_rdy) cout<<"+";
            UpdateState();
            memset(&run_state,0,sizeof(acq_stat));
            }
    }
    board_stop=0;
    board_loop.join();
}

template<> state5742 DigiDevice::GetState<state5742> (){state5742 ret; return ret;}
//void Digitizer::f(){state5742 i = digi.GetState<state5742>();}
