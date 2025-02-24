/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * loader.cpp
 * dirent compile obj */

#include "digitizer.hpp"

#include "digi-gui.hpp"
#include "dirent.h"

//Load files from the assets dir
//files is alphabet-ordered, last overlapped first in GUI layers
void AllGui::LoadSprites(const std::string& path){
    DIR *dir;
    struct dirent *ent;
    if (dir=opendir (path.c_str())){
        vector<string> fnames_p, fnames_l, fnames_b;
        while (ent=readdir(dir))
            if (regex_match (ent->d_name, regex(gui_const::def_fname_regexp))){
                //cout<<regex_replace(ent->d_name, gui_reg, "$1");
                     if (!strncmp(ent->d_name,"Pic",3))
                    fnames_p.push_back(ent->d_name);
                else if (!strncmp(ent->d_name,"Led",3))
                    fnames_l.push_back(ent->d_name);
                else if (!strncmp(ent->d_name,"Btn",3)){
                    fnames_b.push_back(ent->d_name);}
            }

        try{
        map_pic.reserve(fnames_p.size());
        for (string &fname: fnames_p)
            map_pic.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(regex_replace(fname, gui_const::def_fname_regexp, "$1")),
                std::forward_as_tuple(string(path).append(fname)));

        map_led.reserve(fnames_l.size());
        for (string &fname: fnames_l)
            map_led.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(regex_replace(fname, gui_const::def_fname_regexp, "$1")),
                std::forward_as_tuple(string(path).append(fname)));

        map_btn.reserve(fnames_b.size());
        for (string &fname: fnames_b)
            map_btn.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(regex_replace(fname, gui_const::def_fname_regexp, "$1")),
                std::forward_as_tuple(string(path).append(fname)));
        }catch(...){cout<<"sprites load err!\n";}

    closedir (dir);
    } else cout << path <<" open err\n";
}

//labels in text file must be name,label,x,y format each line
void AllGui::LoadTextLabels (const std::string& fname){
    std::ifstream fs(fname, std::ios::in);
    string fl, subs;
    vector<string> vs;

    while (getline(fs, fl)){
        std::stringstream ss(fl);
        while (ss.good()){          
            getline(ss,subs,',');
            subs = regex_replace(subs, regex("^ +| +$|( ) +"), "$1");
            vs.push_back(subs);
        }
    }
    if (vs.size()%4) vs.erase(vs.end()-vs.size()%4);
    map_txt.reserve(vs.size()/4);
    font.loadFromFile("res/SegoeUI.ttf");

    for (auto it=begin(vs); it!=end(vs); it+=4){
        //cout<<"txt_sprite: "<<*it<<"-"<<*(it+1)<<std::endl;
        TxtSprite ts(*it, *(it+1), 
                     atoi((*(it+2)).c_str()), 
                     atoi((*(it+3)).c_str()));
        ts.setFont(font);
        ts.setCharacterSize(gui_const::def_char_size);
        ts.setFillColor    (gui_const::def_font_log_color);
        map_txt.insert({*it,ts});
    }
}

