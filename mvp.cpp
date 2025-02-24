/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * mvp.hpp
 * Model-View-Presenter
 * igor.bocharov@mail.ioffe.ru*/

#include "digitizer.hpp"
#include "mvp.hpp"

void DigiModel::LoopStart(){
        prr.Init();
            FillReg();
            FillRegCoax();
        prr.StartMainLoop();
        thr =  thread(&DigiModel::DeviceMainLoop, this );   // Board loop =>
    }

//cnhg view at open
void DigiModel::ModelDeviceOpen(int state){
    prr.CallView("vgui","BtnOpen",  (state)?1:0);
    prr.CallView("vgui","LedBOpen", (state)?1:0);
}


//Callback from View
void DigiPresenter::CallPresenter(const string &caller, const string &wname, const int val){
    //cout<<"view->pres: "<<caller<<"-"<<wname<<"-"<<val<<std::endl;
        
    //command switcher
    if (wname == app_const::def_glob_signal_cmd){
        switch (val) {
        case app_const::internal_cmd::cmd_halt: //close app
            model->DeviceClose();
            model->LoopStop(); //kill all view threads ==>
            break;
        case app_const::internal_cmd::cmd_kick: //kick sw tr
            char sres[SRES_MAX_LEN];
            caen_reg_write_name( set_5742::tr_soft_kick.c_str(),0,0,0x123,0,sres);
            model->AcqTrSet();
            break;
        case app_const::internal_cmd::cmd_rota: //wheel rorated, deprecated
            break;
        default:break;
        }
    return;
    }

    if (wname == wid_5742::rota_wheel) {
        caen5742_evt *evtp;
        mem_buf_get(&evtp,val);
        model->UpdateSignals(evtp);
        model->DataCalc(evtp);
        return;
    }

    //works forewer
    if (wname == wid_5742::but_board_open) model->DeviceOpenToggle();

    //dont press any Button when board off
    if (model-> AsqIsBReady()==0 && wname.find("Btn") == 0){
        CallView("vgui", wname, 0);
        CallView("vgui", wid_5742::tr_soft_dis, 1);
        }
    //stolb2 (sw trig) back loop in run mode
    else if (wname.find(wid_5742::tr_stolb2_mask) == 0)
        CallView("vgui", wname, val);
    //Coax toggle selector
    else if (wname.find(wid_5742::coax_mode_mark) == 0){ 
        model->curCoaxTab = wname;   //save current tab
        model->ToggleRegCoax(wname); //redraw full coax panel
        }
    else if (wname.find(wid_5742::coax_mark) == 0)  //specific coax but
        model->ToggleRegCoax(wname);
    else
        model->ToggleReg(wname); //main device reg toggler
    
}

//read device reg, merge with settings
void DigiModel::MergeReg (){
    char sval[SVAL_MAX_LEN];
    char sres[SRES_MAX_LEN];
    uint32_t ival=0;

    //load single sets, sets were defined in ctr
    for (auto &set: sets->map_set){
        //BAD check err
        if (!caen_reg_read_name(set.first.c_str(),0,0,&ival,sval,sres)){
            PushLog(sres);
            
            if (set.second.xml==true) //set from xml file
                ToggleReg(set.second.widget, true); //update sets from XML
            else
            { //sets not in xml -> upd view
                if (strlen(sval)>1) {
                    set.second.SetVal(sval); //text labels
                    prr.CallView( gui_const::def_view_name,set.second.widget, sval);
                }else {
                    set.second.SetVal(ival); //buttons
                    prr.CallView( gui_const::def_view_name,set.second.widget, ival);
                }
            }
        }
    }
    prr.Config(wid_5742::cfg_sig, sets->gr_num, sets->ch_num);
    FillRegCoax();
}

//fill gui from xml settings before board open
void DigiModel::FillReg(){
    for (auto &set: sets->map_set)
        prr.CallView(gui_const::def_view_name, set.second.widget, set.second.sval);
}

//find reg by widget name and call update view
//xml=flase - gui call - invert; xml=true - xml call - update view
void DigiModel::ToggleReg(const string &wname, const bool xml){
    //cout<<"ToggleReg="<<wname<<std::endl;

    for (auto it = sets->map_set.begin(); it != sets->map_set.end(); ++it)
        if (it->second.widget == wname){ //find setting by widget name

        ein_set set= it->second;
        cout<<" reg_tog: "<<set.widget<<"->"<<set.name << std::endl;

        char sval[SVAL_MAX_LEN];
        char sres[SRES_MAX_LEN];
        uint32_t ival=0;

        //ask device reg value
        int ret = caen_reg_read_name (set.name.c_str(),0,0,&ival,sval,sres);
        PushLog(sres);
        if (ret==CAENComm_Success){ //read and invert register    
            if ( (xml && (ival != set.ival)) || !xml){ //xml_val != device_val; or call from gui
                ival=(xml)? set.ival : !ival;     //toggle for gui call
                cout<<"invert "<<set.name<<"="<<ival<<std::endl;
                caen_reg_write_name(set.name.c_str(),0,0,ival,0,sres);
                PushLog(sres);
                }
            prr.CallView( gui_const::def_view_name, set.widget, ival);
        }else 
            if (ret==drv_err_wo){  //only write register
            if (caen_reg_write_name(set.name.c_str(),0,0,ival,0,sres) == CAENComm_Success)
                prr.CallView( gui_const::def_view_name,set.widget, true); //call true
                PushLog(sres);
            }
        else //something wrong
            {prr.CallView( gui_const::def_view_name,set.widget, false);
            }
    }
}

//gr0-1.ch0-7  ==>  BtnCoaxCh00-07,10-17
//gr0-1.ch*    ==>  BtnCoaxGr0-1
//gr*.ch*      ==>  BtnCoaxTr0
string ch2coax(const int gr, const int ch){
    //cout<<"chs:gr"<<std::to_string(gr)<<".ch"<<std::to_string(ch)<<" ";
    string coax_name;

    //gr=0-1 ch=0-7 -> 00-07,10-17
    if (gr<set_5742::ch_all && ch<set_5742::ch_all)
        coax_name = wid_5742::coax_ch_mark + std::to_string(gr) +  std::to_string(ch);
    //gr=0-1, ch=all
    else if (gr<set_5742::ch_all && ch==set_5742::ch_all) 
        coax_name = wid_5742::coax_gr_mark + std::to_string(gr);
    //gr=all, ch=all
    else if (ch==set_5742::ch_all && gr==set_5742::ch_all)
        coax_name = wid_5742::coax_tr_mark + "0";

    return coax_name;
}

//fill gui from xml settings before board open
void  DigiModel::FillRegCoax(){
    cout<<"FillRegCoax"<<std::endl;
    for (ch_set &chs: sets->vec_set){
        string coax_name = ch2coax(chs.gr, chs.ch);
        if (curCoaxTab == wid_5742::tab_view)
            prr.CallView("vgui", coax_name, (chs.view == set_5742::tag_empty)? 1 : chs.view );
        else cout<<" no_fill ";
        //if (curCoaxTab == "Mask" && chs.mask != set_5742::ch_all) 
        //    prr.CallView("vgui", coax_name, chs.mask);
    }
}

//find CoaxBut in vector and toggle it
void  DigiModel::ToggleRegCoax(const string &tname, const bool xml){
    //cout<<"ToggleRegCoax="<<curCoaxTab<<"."<<tname<<", ";//
    char sval[SVAL_MAX_LEN]; sval[0]=0;
    char sres[SRES_MAX_LEN]; //cout<<"~~~"<<ival<<"~~~";
    uint32_t ival=0; int ret;
    
    for (ch_set &chs: sets->vec_set){   //find coaxbut in vector
        string coax_name = ch2coax(chs.gr, chs.ch);
        //cout<<"cx_n="<<coax_name<<" ";
        if (coax_name == tname){           
            //if (chs.gr < set_5742::ch_all && chs.ch < set_5742::ch_all){
            
            if (curCoaxTab == wid_5742::tab_view){ //save view settings
                if (chs.view == set_5742::tag_empty || chs.view == 0)  chs.view = 1;
                    else chs.view = 0;
                prr.CallView("vgui", coax_name, chs.view); //already toggled
            }
            else if (curCoaxTab == wid_5742::tab_mask){ cout<<" MSK ";
                //gr0 gr1
                if (tname.find(wid_5742::coax_gr_mark) == 0){
                    ret = caen_reg_read_name ("sel_gr",0,0,&ival,sval,sres);
                    PushLog(sres);
                    ival ^= 1UL<<chs.gr; //toggle
                    ret = caen_reg_write_name ("sel_gr",0,0,ival,0,sres);
                    PushLog(sres);                        
                    ival =  (ival >> chs.gr) & 1U; //for gui
                    }
                //tr
                else if (tname.find(wid_5742::coax_tr_mark) == 0){
                    ret = caen_reg_read_name  ("tr_fast_dgtize",0,0,&ival,sval,sres);
                    PushLog(sres);
                    ival=!ival;
                    ret = caen_reg_write_name ("tr_fast_dgtize",0,0,ival,0,sres);
                    PushLog(sres);
                    }
                prr.CallView("vgui", coax_name, ival);
            }    
            else if (curCoaxTab == "Dc")
                ;

            else if (curCoaxTab == "Tr")
                ;
            //}
        }
    }
}

//update LED status
void DigiModel::UpdateState(){
    prr.CallView( gui_const::def_view_name, wid_5742::led_open, run_state.board_rdy);
    prr.CallView( gui_const::def_view_name, wid_5742::led_run,  run_state.acq_run);
    prr.CallView( gui_const::def_view_name, wid_5742::led_trg,  run_state.trg_in);
    prr.CallView( gui_const::def_view_name, wid_5742::led_rdy,  run_state.evt_rdy);
    prr.CallView( gui_const::def_view_name, wid_5742::led_busy, run_state.evt_full);
    prr.CallView( gui_const::def_view_name, wid_5742::led_pll,  run_state.pll_unlck);
}

void DigiModel::UpdateSignals(const caen5742_evt *evt){
     
    for (uint8_t g=0; g<sets->gr_num; g++){
        prr.CallView(gui_const::def_view_name, 
            (wid_5742::sig_tr+std::to_string(g) ), 
            evt->evt_buf[g][sets->ch_num]);
    for (uint8_t c=0; c<sets->ch_num; c++)
        prr.CallView(gui_const::def_view_name, 
            (wid_5742::sig_ch+std::to_string(c+ ((g)?sets->ch_num:0) ) ), 
            evt->evt_buf[g][c]);
    }
}

void DigiModel::DataCalc(const caen5742_evt *evtp){
    TSalgo tsa(*evtp);
    prr.CallView(gui_const::def_view_name, wid_5742::calc_T, tsa.Te);
    prr.CallView(gui_const::def_view_name, wid_5742::calc_N, tsa.Ne);
}