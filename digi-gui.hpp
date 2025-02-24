/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * digi-gui.hpp
 * digitizer GUI 
 * igor.bocharov@mail.ioffe.ru*/

#pragma once

#include <SFML/Graphics.hpp>

namespace gui_const{
    const string def_view_name   = "vgui"; //gui widget View name

    //regexp "WidgetName-XPOSxYPOS.jpg"
    const regex  def_fname_regexp("(.*)-(\\d{1,4})x(\\d{1,4})(.png|.jpg|.gif)$"); 
    const string def_res_folder  = "./res/"; //folder for resourses
    const string def_labels_file = "./res/labels.txt"; //filename with text labels

    const sf::Color def_font_log_color      = sf::Color(134,151,151); //default font color
    const sf::Color def_font_log_color_err  = sf::Color(200,100,100); //error msg
    const unsigned int def_char_size = 20; //default font size

    const unsigned int def_logl_len  = 30; //gui log line max length
    const unsigned int def_ats_size  = 5;  //gui log widget lines
    const unsigned int def_als_size  = 12; //gui log buffer size

    //big wheel position and radius
    const unsigned int def_wheel_xpos = 1095;
    const unsigned int def_wheel_ypos =  246;
    const unsigned int def_wheel_size = 120;    
    const unsigned int def_wheel_pipr =  25;

    //arc widgets, two pcs 
    const unsigned int def_arcT_xpos  = 1302;
    const unsigned int def_arcN_xpos  = 1426;
    const unsigned int def_arc_ypos   = 261;
    const unsigned int def_arc_radius = 46;
    const float        def_arc_thick  = 1.5;
}

//simple static picture loader
class PicSprite: public sf::Sprite{
    private:
    sf::Vector2u pz;

    protected:
    sf::Texture  txtr; 
    string name;       //sprite name for MVP

    public:
    /*PicSprite(){cout<<"ctor ";}
    PicSprite(const PicSprite &){cout<<"c-ctor ";}
    PicSprite(PicSprite &&) {cout<<"m-ctor ";}*/

    string GetName(){return name;}

    PicSprite(const std::string& filename){
        //std::cout << "load_sprite: " <<filename<<std::endl;
        txtr.loadFromFile(filename);
        setTexture(txtr);
        name = filename.substr(filename.find_last_of("/\\") + 1);
        name =           (std::regex_replace(    name, gui_const::def_fname_regexp, "$1"));
        pz.x = std::stoi (std::regex_replace(filename, gui_const::def_fname_regexp, "$2"));
        pz.y = std::stoi (std::regex_replace(filename, gui_const::def_fname_regexp, "$3"));
        setPosition(sf::Vector2f(pz));        
    }
};

//toggle texture functionality
class LedSprite: public PicSprite{   
    public:
    LedSprite(const std::string& filename):PicSprite(filename){
        sz = txtr.getSize();
        SetOff();
    }
    
    void SetOn () {state=1; setTextureRect(sf::IntRect(0, 0       , sz.x, sz.y>>1));}
    void SetOff() {state=0; setTextureRect(sf::IntRect(0, sz.y>>1 , sz.x, sz.y>>1));}
    void Toggle() { if (state=!state) SetOn(); else SetOff();}
    void SetState(int set) { if (set>0 && set!=0xFFFF) SetOn(); else SetOff();}//BAD
    int  GetState(){return state;}

private:
    sf::Vector2u sz;
    bool state;
};

//toggle button functionality
class BtnSprite: public LedSprite{
    public:
    BtnSprite(const std::string& filename):LedSprite(filename){}

};

//text in x,y location
class TxtSprite: public sf::Text{
    public:
    TxtSprite(const string& name_,
              const string& label_,
              const int x, const int y):
              name(name_),label(label_){
        setPosition(sf::Vector2f(x,y));
        setString  (label);
    }

    void SetText(const std::string& txt){
        setString  (label+" "+txt);
        }

    private:
    string name;
    string label;
};

//Rotary wheel class
//wheel texture on substrate, only "pipka" sprite rotates
class RotaWheel: public sf::Drawable, public sf::Transformable{
public:
    RotaWheel (const string name, const int offsX, const int offsY, 
               const int sen=25, const int linmax=1000);

    void Rotate(const float angl);
    void Rotate(const sf::Vector2i mouse_pos);
    float GetTurn() {return da;} //delta angle
    int GetLinPos() {return linpos;} //linear position 0...maxlin

    string name;
    string GetName(){return name;}
    bool mcatch; //mouse catched

    sf::RectangleShape rect; //for getGlobalBounds
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    int offsX;
    int offsY;

    sf::Sprite pipka;
    sf::Texture pipt;

    float oa;  //old angle
    float da;  //delta angle

    int linpos; //current linear position
    int linmax; //max linear position
    int sen;    //wheel sensitivity
};

//Arc gauge class
class ArcGauge: public sf::Drawable, public sf::Transformable{
public:
    ArcGauge();
    ArcGauge(const string name_, int offsX_, int offsY, int radius_, float thick_, sf::Color colora_, sf::Color colorf_);
    void SetVal(int val); //val int 0-100
    sf::Font font;
    sf::Text txt;
    static const sf::Color color_arcT;
    static const sf::Color color_arcN;
    static const sf::Color color_text;

    string name;
    string GetName(){return name;}

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    sf::VertexArray chunks;
    sf::CircleShape circ;
    
    
    sf::Color colora;
    sf::Color colorf;
    float thick;
    int offsX;
    int offsY;
    int radius;
    int val;
};


//logger for gui
class GuiLogger{
public:
    array<sf::Text,5> ats;  //array of text sptites
    GuiLogger();
    void PushLogLine(const string& line);
    void IncReadShift(){ if (curS < als_size) curS++;}
    void DecReadShift(){ if (curS >0)         curS--;}

private:
    static constexpr const int als_size = gui_const::def_als_size;
    static constexpr const int ats_size = gui_const::def_ats_size;
    array<string,als_size>  als;  //array of log strings
    int curW; //write cursor
    int curR; //read cursor
    int curS; //shift read cursor
    sf::Font font;
    const sf::Color color_norm = sf::Color(134,151,151); //norm msg
    const sf::Color color_err  = sf::Color(200,100,100); //error msg

    const string err_marker; //Error marker inside string
    
    const array<sf::Vector2f,ats_size> line_pos{
        {sf::Vector2f(1058, 663),
         sf::Vector2f(1058, 690),
         sf::Vector2f(1058, 717),
         sf::Vector2f(1058, 744),
         sf::Vector2f(1058, 771)}
         };  //array of log strings
};

//Signal cursor, mouse draggable
class OscCursorShape : public sf::Drawable, public sf::Transformable{
public:
    enum cur_win { cur_ch_dc, cur_ch_th, cur_tr_dc, cur_tr_th }; //cursor window

    OscCursorShape(const cur_win cw_, const int val_, const int sz_);
  
    void SetVal(const int val_){
        int val=val_;
        if (val<0)   val=0;
        if (val>100) val=100;
         SetCurPosAbs(ymax-( val * (ymax-ymin) /100 ));
    }

    int GetVal(){return ( 100*(ymax-ypos)/(ymax-ymin));}

    void SetCurPosRel(const int pos_){
         SetCurPosAbs(ymax-pos_);
    }

    void SetCurPosAbs(const int pos_){
        ypos = pos_;
        if (pos_<ymin) ypos=ymin;
        if (pos_>ymax) ypos=ymax;
             setPosition(xpos,     ypos-sz);
        rect.setPosition(xpos-sz*3,ypos-sz);
    }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const{
        states.transform *= getTransform();
        target.draw(va, states);
    }
    
    string name;  //widget name
    bool mcatch; //mouse catched
    sf::RectangleShape rect; //for getGlobalBounds

private:
    sf::VertexArray va; //vertices array
    sf::Color clr;      //vertices color
    int sz;     //widget size

    int ypos;   //widget Y pos abs
    int ymin;   //signal window top ypos
    int ymax;   //signal window bottom ypos

    const int xpos = 1034; //signal window right side x pos
    const int xlen = 1024; //signal window width
};

//signal chart shape
class SignalShape : public sf::Drawable, public sf::Transformable{
public:
    typedef enum { sig_ch, sig_tr} sig_win; //signal window

    SignalShape(const sig_win sw, const int num, const bool is_draw, const int sszie);
    string name;    //widget name
    bool is_draw;   //draw or not to draw

    void SetData (const uint16_t *data){
        for (int i=0; i<ssize; i++){
            va[i].position = sf::Vector2f(i,-(data[i]>>yamp));
            va[i].color = clr;
        }
    }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const{
        states.transform *= getTransform();
        if (is_draw) target.draw(va, states);
    }
    
private:
    sig_win sw;     //signal window type: cannel or trigger
    sf::VertexArray va;     //array of vertex
    const int ssize = 1024; //signal size fix
    int yamp;       //y axis amplification in shift bits from uint16_t 12b
    int xoffs;      //window X offset
    int yoffs;      //window Y offset
    sf::Color clr;  //signal color
};

//all gui elements
class AllGui: public DigiView{
    public:
    AllGui(); //Like to construct gui by separately functions

    //from virtual interface
    void Init();
    void Config(const string &param, const int val1, const int val2);
    void MainLoop();
    void CallView(const string &param, const int val);
    void CallView(const string &param, const string &val);
    void CallView(const string &param, const uint16_t val[]);
    void PushLog(const std::string& txt);

    //local functions
    void SetWindowTitle (const std::string& fname){win_title="GlobusTS "+fname;}
    void LoadSprites    (const std::string& path  = gui_const::def_res_folder );
    void LoadTextLabels (const std::string& fname = gui_const::def_labels_file);
    void MainWindowLoop ();

    unordered_map<string,PicSprite> map_pic; //simple pictures
    unordered_map<string,LedSprite> map_led; //toggles pictures aka Led
    unordered_map<string,BtnSprite> map_btn; //toggles buttons
    unordered_map<string,TxtSprite> map_txt; //simple text labels
    vector<OscCursorShape> vcur;             //signal cursors
    vector<SignalShape> vsign;               //signal charts
   
    GuiLogger gl; //log to gui widget
    
    //plasma temperature arc-gauge widget
    ArcGauge agT=ArcGauge( "ArcGaugeT",
        gui_const::def_arcT_xpos,gui_const::def_arc_ypos,
        gui_const::def_arc_radius,gui_const::def_arc_thick,
        ArcGauge::color_arcT,ArcGauge::color_text);
    
    //plasma density arc-gauge widget
    ArcGauge agN=ArcGauge( "ArcGaugeN",
        gui_const::def_arcN_xpos,gui_const::def_arc_ypos,
        gui_const::def_arc_radius,gui_const::def_arc_thick,
        ArcGauge::color_arcN,ArcGauge::color_text);
    
    //rotary wheel widget
    RotaWheel rw=RotaWheel("RotaWheel",
        gui_const::def_wheel_xpos,
        gui_const::def_wheel_ypos); 

    private:
    sf::Font font;      //main app font
    string win_title;   //config filename for title
    void LoadAux();     //load auxulary widgets
    void Toggler(const string& sprt); //toggle widget from model
    void Downer();      //turn down by timer

    int ch_num;
    int gr_num;

    std::unordered_map<string,BtnSprite>::iterator tab_it; //active tab
};
