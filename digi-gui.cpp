/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * digitizer GUI
 * entry point */

#include "digitizer.hpp"
#include "digi-gui.hpp"

//construct logger window
GuiLogger::GuiLogger():curR(0),curW(0),curS(0),
           err_marker(app_const::def_con_err_marker),
           color_norm(gui_const::def_font_log_color),
            color_err(gui_const::def_font_log_color_err){

    font.loadFromFile("res/SegoeUI.ttf");
    
    sf::Text text; //default test style for log
    text.setFont(font);
    text.setCharacterSize(gui_const::def_char_size);
    text.setFillColor(color_norm);
    text.setString("");
    ats.fill(text);

    auto it_pos = line_pos.begin()-1;//set lines position
    for(auto &sprt: ats)
        sprt.setPosition(*++it_pos);
}

//ring buffer for write log and read, with shift
//     v-CurW
//[0 1 2 3 4 5 6 - - - - als_sz]
//  -CurS-[^-CurR---alt_sz]  
void GuiLogger::PushLogLine(const std::string& line){

    std::time_t t =  std::time(0); //insert time
    std::tm   now = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&now, "%H:%M:%S");
    auto str = oss.str();
    string ins_line (oss.str()+" "+line.substr(0,gui_const::def_logl_len));

    als.at(curW)=ins_line;       //put string in to buffer

    if (curW<als_size-1) curW++; //incr buffer write counter
                    else curW=0;

    if (curW<curS) curR=curW+(als_size-curS); //shift read counter !!!!-1 both
              else curR=curW-curS;

    auto it_txt = als.begin();  //set read iterator
    if (curR<ats_size) it_txt = next(it_txt,curR+(als_size-ats_size));
                  else it_txt = next(it_txt,curR-ats_size);
   
    for(auto &sprt: ats){ //fill text sprite array
        sprt.setString(*it_txt);

        if (it_txt->find(err_marker) != std::string::npos) //Error colorer
                 sprt.setFillColor(color_err);
        else if (sprt.getFillColor()==color_err) //srop old err color
                 sprt.setFillColor(color_norm);

             it_txt =  next(it_txt,1);
        if ( it_txt == als.end())
             it_txt =  als.begin();
    }
}

const sf::Color ArcGauge::color_arcT = sf::Color(250,240, 50);
const sf::Color ArcGauge::color_arcN = sf::Color(250,100,  0);
const sf::Color ArcGauge::color_text = sf::Color(220,250,166);

ArcGauge::ArcGauge(){}

ArcGauge::ArcGauge(const string name_, int offsX_, int offsY, int radius_, float thick_,
                   sf::Color colora_, sf::Color colorf_):
        name(name_),
        offsX(offsX_),
        offsY(offsY),
        radius(radius_),
        thick(thick_),
        colorf(colorf_),
        colora(colora_){
    circ.setRadius(thick);
    circ.setPosition(sf::Vector2f(offsX-thick,offsY+radius-thick));
    circ.setOutlineColor(colora);
    circ.setFillColor   (colora);
    circ.setOutlineThickness(thick);

    font.loadFromFile("res/SegoeUI.ttf");  
    txt.setFont(font);
    txt.setStyle(sf::Text::Bold);
    txt.setCharacterSize(gui_const::def_char_size);
    txt.setFillColor(colorf);
    txt.setPosition(sf::Vector2f(offsX, offsY-7));

    SetVal(0);
}


void ArcGauge::draw(sf::RenderTarget& target, sf::RenderStates states) const{
        target.draw(circ,   states);
        target.draw(chunks, states);
        target.draw(txt,    states);
    }

void ArcGauge::SetVal(int val){
    if (val<0) val=0; if (val>100) val=100;
    if (val%2) val++; //make even for TriangleStrip

    txt.setString(std::to_string(val));
    sf::FloatRect textRect = txt.getLocalBounds();
    txt.setOrigin(textRect.left + textRect.width/2.0f,
                  textRect.top  + textRect.height/2.0f);

    chunks=sf::VertexArray(sf::TriangleStrip, val+2);
    float x,y,si,ci;
    for (float i = 0; i < val+2; i+=2){
        si = -sin(i*0.0628);
        ci =  cos(i*0.0628);

        x = offsX + (radius-thick) * si;
        y = offsY + (radius-thick) * ci;
        chunks[i].position = sf::Vector2f(x,y);
        chunks[i].color = colora;

        x = offsX + (radius+thick) * si;
        y = offsY + (radius+thick) * ci;
        chunks[i+1].position = sf::Vector2f(x,y);
        chunks[i+1].color = colora;
        }
}


RotaWheel::RotaWheel(const string name_, 
                     const int offsX_, const int offsY_, 
                     const int sen_, const int linmax_):
        name(name_),
        offsX(offsX_),
        offsY(offsY_),
        sen(sen_),
        linmax(linmax_),
        mcatch(false),
        linpos(0){
    pipt.loadFromFile("res/pipka.png");
    pipka.setTexture(pipt);
    Rotate(0);

    int sz = gui_const::def_wheel_size;
    rect = sf::RectangleShape(sf::Vector2f(sz, sz));
    rect.setPosition(sf::Vector2f(offsX-sz/2, offsY-sz/2));
}

void RotaWheel::Rotate(const sf::Vector2i mouse_pos){
    int dx = mouse_pos.x - offsX;
    int dy = mouse_pos.y - offsY;
    float a  = atan2(dy, dx);
    Rotate(a);
}

void RotaWheel::Rotate(const float angl){
    //cout<<" a="<<std::to_string(angl)<<" ";
    pipka.setPosition(sf::Vector2f(
        offsX + gui_const::def_wheel_pipr * cos(angl),
        offsY + gui_const::def_wheel_pipr * sin(angl)));
    da = angl-oa; //positive = clockwise
    if (da>1 || da<-1) da=0; // transition I<->IV quadrant
    oa = angl;

    //convert to linear scale 0...maxlin
    int t = da * sen;
    if (da> 0.01 && t< 5) t= 1;
    if (da<-0.01 && t>-5) t=-1;
    if ( linpos+t>=0 || (linpos==0 && t>0)) linpos+=t;
    if (linpos>linmax) linpos=linmax;
}

void RotaWheel::draw(sf::RenderTarget& target, sf::RenderStates states) const{
    target.draw(pipka,states);
}

OscCursorShape::OscCursorShape(
    const OscCursorShape::cur_win cw_,      //channel or trigger window
    const int val_=0,       //start value poisition 0-100
    const int sz_=8):       //cursor size
    sz(sz_),mcatch(0){
    switch(cw_){
        case cur_ch_dc: 
            ymin=10;  ymax=512+ymin;
            name=wid_5742::cur_chdc;
            clr = sf::Color(250, 150, 100, 180);
            break;
        case cur_ch_th:
            name=wid_5742::cur_chth;
            clr = sf::Color(220, 140, 120, 160);
            break;
        case cur_tr_dc: 
            ymin=546; ymax=254+ymin;
            name=wid_5742::cur_trdc;
            clr = sf::Color(150, 60, 250, 220);
            break;
        case cur_tr_th:
            ymin=546; ymax=254+ymin;
            name=wid_5742::cur_trth;
            clr = sf::Color(120, 50, 220, 200);
            break;
    };

    //cursor triangle draw form
    va.setPrimitiveType(sf::TriangleStrip);
    va.resize(8);
        va[0].position = sf::Vector2f(0    ,0   );
        va[1].position = sf::Vector2f(0    ,sz*2);
        va[2].position = sf::Vector2f(-sz*2,0   );
        va[3].position = sf::Vector2f(-sz*2,sz*2);
        va[4].position = sf::Vector2f(-sz*3,sz  );
        va[5].position = sf::Vector2f(-sz*3,sz+1);
        va[6].position = sf::Vector2f(-xlen,sz  );
        va[7].position = sf::Vector2f(-xlen,sz+1);
    for (std::size_t i=0; i <va.getVertexCount(); ++i) va[i].color= clr;
    
    rect = sf::RectangleShape(sf::Vector2f(sz*3,sz*2));

    SetVal(val_);
}

SignalShape::SignalShape(
    const sig_win sw_,  //window type channel or trigger
    const int num,      //signal number
     const bool is_draw_ = true, //draw curve by default
    const int sszie_=CAEN_DT7542_SIGWIN  //length of the signal
    ):ssize(sszie_), is_draw(is_draw_){   
        
    switch(sw_){ 
            case sig_ch: 
                xoffs=10;  yoffs=521;
                name=wid_5742::sig_ch+std::to_string(num);
                clr = sf::Color(90+num*10, 255, 255-num*10);
                yamp=3; //12b->512
                break;

            case sig_tr: 
                xoffs=10;  yoffs=800;
                name=wid_5742::sig_tr+std::to_string(num);
                clr = sf::Color(10+num*100, 255, 250-num*100);
                yamp=4; //12b->256
                break;
    };

    va.setPrimitiveType(sf::LineStrip);
    va.resize(ssize);
        for (int i=0; i<ssize; i++){
            va[i].position = sf::Vector2f(i, -1);//-num*10-10);
            va[i].color = clr;
            }
    this->setPosition(sf::Vector2f(xoffs,yoffs));
}

//------------------------------------------------------------------------------
AllGui::AllGui():
    DigiView(gui_const::def_view_name),
    ch_num(0),gr_num(0){
}

void AllGui::Init(){
    LoadSprites   (gui_const::def_res_folder );
    LoadTextLabels(gui_const::def_labels_file);
    LoadAux();

    //set first tab
    if ( (tab_it = map_btn.find(wid_5742::tab_view)) != map_btn.end() ) 
        tab_it->second.SetOn();
}

void AllGui::Config(const string &param, const int val1,  const int val2){
    //cout<<"gui_config: "<<param<<"-"<<val1<<","<<val2<<std::endl;
    
    //create signals veсtor, gr * channels
    if (param.find(wid_5742::cfg_sig) != string::npos){
        vsign.clear();
        vsign.reserve(val1*(val2+1)); //2*(9+1)
        gr_num =val1; ch_num = val2;

        for (int i=0; i<val1; i++)          
            vsign.emplace_back( SignalShape::sig_tr, i,
                map_btn.find(wid_5742::coax_tr_mark + "0")->second.GetState());

        for (int i=0; i<val2*val1; i++){ //ch0-7, ch8-15
            string wname;
            if (i>7) wname = wid_5742::coax_ch_mark + std::to_string(i+2); //0-7,10-17
                else wname = wid_5742::coax_ch_mark + "0" + std::to_string(i);
            bool e = map_btn.find(wname)->second.GetState();
            vsign.emplace_back( SignalShape::sig_ch, i, e);
            }
    }
}

void AllGui::CallView(const string &param, const int val){
    //if (param.find("Led") != 0)
        //cout<<"pres->view: "<<param<<"-"<<val<<std::endl;

    auto btn = map_btn.find(param);
    if (btn != map_btn.end()) {
        btn->second.SetState(val);
        Toggler(btn->second.GetName()); //post update widgets
        }
    
    auto led = map_led.find(param);
    if (led != map_led.end()) led->second.SetState(val);

    if (param==wid_5742::calc_T) agT.SetVal(val);
    if (param==wid_5742::calc_N) agN.SetVal(val);
}

void AllGui::CallView(const string &param, const string &val){
    //cout<<"pres->view_s: "<<param<<"-"<<val<<std::endl;

    auto txt = map_txt.find(param);
    if (txt != map_txt.end()) txt->second.SetText(val);
}

void AllGui::CallView(const string &param, const uint16_t val[]){
    //cout<<"pres->view[]: "<<param<<std::endl;
    
    if (param.find(wid_5742::sig_ch) != string::npos){
        for (int i=0; i<(ch_num+1)*gr_num; i++)
            if (vsign[i].name == param){
                vsign[i].SetData((const uint16_t*)val); }

    }
    if (param.find(wid_5742::sig_tr) != string::npos){
        for (int i=0; i<(ch_num+1)*gr_num; i++)
            if (vsign[i].name == param)
                vsign[i].SetData((const uint16_t*)val);
    }
}

//Toggle widgets state from model
void AllGui::Toggler(const string& wname){
    //cout<<"Toggler "<<wname<<"\n";
    
    if ( map_btn.find(wname) == map_btn.end() ) return;
    auto &sprt = map_btn.find(wname)->second;

    //Coax giant toggler
    if (wname.find(wid_5742::coax_mark) == 0){  //coax panel but "BtnCoax"
        
        //Active tab toggler
        if (wname.find(wid_5742::coax_mode_mark) == 0){
            if (tab_it != map_btn.end()) tab_it ->second.SetOff();
            tab_it = map_btn.find(wname);
            if (tab_it != map_btn.end()) tab_it->second.SetOn();
        }
        
        //TAB VIEW mode - on/off signal curve in GUI
        if (map_btn.find(wid_5742::tab_view)->second.GetState()){

            //tr
            if (wname.find( (wid_5742::coax_tr_mark) ) == 0){
                for (auto &s: vsign)
                    if (s.name.find(wid_5742::sig_tr) == 0) s.is_draw = sprt.GetState();
            } 
            else //ch
            if (wname.find( (wid_5742::coax_ch_mark) ) == 0){ 
                for (auto &s: vsign){
                    //Ch0-17 -> CoaxButCh00-07,10-17
                    int ch_num = std::stoi( wname.substr(wid_5742::coax_ch_mark.size()) );
                    if (ch_num>7) ch_num-=2; 
                    int  ch_ch = std::stoi( s.name.substr(wid_5742::sig_ch.size()) );
                    if (s.name.find(wid_5742::sig_ch)==0 &&
                        ch_num == ch_ch ) //cout<<sprt.GetState();
                            s.is_draw = sprt.GetState();
                }
            }
            else //gr
            if (wname.find( (wid_5742::coax_gr_mark) ) == 0){ 
                int gr_num = std::stoi( wname.substr(wid_5742::coax_gr_mark.size()) );
                for (auto &s: vsign){
                    int ch_num = std::stoi( s.name.substr(wid_5742::sig_ch.size()) );
                    if (s.name.find(wid_5742::sig_ch) == 0 &&
                        ch_num >= 8*gr_num && ch_num < 8+gr_num*8 ) 
                            s.is_draw = sprt.GetState();
                }
            }
        }else //TAB MASK - off coax_ch
        if (map_btn.find(wid_5742::tab_mask)->second.GetState()){ 
            for (auto &btn: map_btn)
                if ( btn.second.GetName().find( (wid_5742::coax_ch_mark) ) == 0)
                     btn.second.SetOff();
        }else if (map_btn.find(wid_5742::tab_dc  )->second.GetState()){ //cout<<" tab_dc";
        }else if (map_btn.find(wid_5742::tab_tr  )->second.GetState()){ //cout<<" tab_tr";
        }
    }
    
    //SW trigger activator -----
    //never touch overlapped but
    if (wname == wid_5742::tr_soft_dis) sprt.Toggle();
    //proseed kick/cont block
    vector<string> swtr_names={wid_5742::tr_soft_kick, wid_5742::tr_soft_cont, wid_5742::tr_soft_off, wid_5742::tr_soft};
    if (std::find(swtr_names.begin(), swtr_names.end(), wname) != swtr_names.end()){ cout<<" kick_tgl "<<wname<<" ";
        BtnSprite *ss = &map_btn.find(wid_5742::tr_soft)     ->second;
        BtnSprite *sd = &map_btn.find(wid_5742::tr_soft_dis) ->second;
        BtnSprite *sk = &map_btn.find(wid_5742::tr_soft_kick)->second;
        BtnSprite *so = &map_btn.find(wid_5742::tr_soft_off) ->second;
        BtnSprite *sc = &map_btn.find(wid_5742::tr_soft_cont)->second;
        if (ss->GetState()){
            sd->SetOn();    //dis aprite to alpha channel
            bool sos = so->GetState();
                 if (wname == wid_5742::tr_soft)      so->SetOn(); //kick/cont activated
            else if (wname == wid_5742::tr_soft_kick) {if (sc->GetState()) sc->SetOff(); so->SetOff();}
            else if (wname == wid_5742::tr_soft_off ) {if (sk->GetState()) sk->SetOff(); sc->SetState(!sos);}
            else if (wname == wid_5742::tr_soft_cont) {if (sk->GetState()) sk->SetOff(); so->SetState(!sos);}
        }else{//disable kisk/cont block
            sd->SetOff(); sk->SetOff(); so->SetOff(); sc->SetOff();}
    }
}

void AllGui::Downer(){
    if (map_btn.find(wid_5742::tr_soft_kick)->second.GetState()){
        map_btn.find(wid_5742::tr_soft_kick)->second.SetState(false);
        map_btn.find(wid_5742::tr_soft_off)->second.SetState(true);
    }

    if (map_btn.find(wid_5742::tr_soft_cont)->second.GetState())
        CallPres(app_const::def_glob_signal_cmd, app_const::internal_cmd::cmd_kick);
}

void AllGui::LoadAux(){
    //Load signal cursors
    vcur.push_back(OscCursorShape(OscCursorShape::cur_ch_dc, 40));
    vcur.push_back(OscCursorShape(OscCursorShape::cur_ch_th, 95));
    vcur.push_back(OscCursorShape(OscCursorShape::cur_tr_dc, 30));
    vcur.push_back(OscCursorShape(OscCursorShape::cur_tr_th, 80));

    for (auto &ocur: vcur){
        auto txt = map_txt.find(ocur.name);
        if (txt != map_txt.end()) txt->second.SetText(std::to_string(ocur.GetVal()));
        }
}

void AllGui::PushLog(const std::string& txt){gl.PushLogLine(txt);}

void AllGui::MainLoop(){ MainWindowLoop(); } //proxy

//------------------------------------------------------------------------------
//GUI main loop

void AllGui::MainWindowLoop(){
    sf::RenderWindow window(sf::VideoMode(1500, 810), win_title);
    window.setVerticalSyncEnabled(true);

    sf::Texture texture_substrate;
    texture_substrate.loadFromFile("res/substrate.png");
    sf::Sprite substrate;
    substrate.setTexture(texture_substrate);
    substrate.setPosition(sf::Vector2f(0, 0));
    
    sf::Vector2f  mice_pos;
    sf::Event mice_evt;
    sf::Clock clock;    //main clock
    sf::Time widg_delay(sf::milliseconds(500));

    int pos = 0;

    while (window.isOpen()){
        
        while (window.pollEvent(mice_evt)){
            mice_pos=window.mapPixelToCoords(sf::Mouse::getPosition(window));
            if (mice_evt.type == sf::Event::Closed)
                window.close();                                 //==> Exit (thread)

            if (mice_evt.mouseButton.button == sf::Mouse::Left){
                
                if (mice_evt.type == sf::Event::MouseButtonPressed){ //mouse pressed
                    for (auto &sprt: map_btn)
                        if (sprt.second.getGlobalBounds().contains(mice_pos)) //button widget
                            {sprt.second.Toggle(); //toggle ANY widget
                             CallPres(sprt.second.GetName(), sprt.second.GetState()); //call model ==>
                            }//Toggler (sprt.second.GetName(),1);
                    for (auto &ocur: vcur)
                        if (ocur.rect.getGlobalBounds().contains(mice_pos))   //cursor
                                ocur.mcatch=true;
                    if (rw.rect.getGlobalBounds().contains(mice_pos))
                            rw.mcatch = true;
                }
                if (mice_evt.type == sf::Event::MouseButtonReleased){          //mouse released
                        for (auto &ocur: vcur)
                            if (ocur.mcatch)
                                ocur.mcatch=false;
                    if (rw.mcatch) rw.mcatch=0;
                    cout<<std::endl;
                }
            }
        }

        window.clear(sf::Color::Black);
        window.draw(substrate);

        for (auto &sprt: map_pic)
            window.draw(sprt.second);

        for (auto &sprt: map_led)
             window.draw(sprt.second);

        for (auto &sprt: map_btn)
             window.draw(sprt.second);

        for (auto &sprt: map_txt)
             window.draw(sprt.second);

        for (auto &sprt: gl.ats)
             window.draw(sprt);

        for (auto &sign: vsign)
             window.draw(sign);

        for (auto &ocur: vcur){  //oscil cursors
             if (ocur.mcatch){
                ocur.SetCurPosAbs(window.mapPixelToCoords(sf::Mouse::getPosition(window)).y);
                auto txt = map_txt.find(ocur.name);
                if (txt != map_txt.end()) txt->second.SetText(std::to_string(ocur.GetVal()));
                }
             window.draw(ocur);
            }

        if (rw.mcatch) {
            rw.Rotate(sf::Mouse::getPosition(window));
            CallPres(rw.GetName(), rw.GetLinPos());
            auto txt = map_txt.find("MemBufPos");
            if (txt != map_txt.end())
                txt->second.SetText(std::to_string(rw.GetLinPos()));
            }

        window.draw(rw);                             

        window.draw(agT);
        window.draw(agN);

        if (clock.getElapsedTime()>widg_delay){
                clock.restart();
                Downer();
            }

        window.display();
        }
    //here is the end ==>
    CallPres(app_const::def_glob_signal_cmd, app_const::internal_cmd::cmd_halt);
}
//std::string::npos
//END---------------------------------------------------------------------------