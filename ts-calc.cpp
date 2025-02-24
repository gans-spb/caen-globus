/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * ts-calc.cpp
 * Thomson scattering calculator
 * igor.bocharov@mail.ioffe.ru*/

#include "digitizer.hpp"

TSalgo::TSalgo(const caen5742_evt &evt_):evt(evt_){
    Calc();
}

void TSalgo::Calc(){
    std::vector<int32_t> avgch;
    avgch.reserve(CAEN_DT7542_CC_NUM);
    uint32_t acu=0, bias;
    int32_t avg;
    for (uint16_t g=0; g<CAEN_DT7542_GR_NUM; g++){
        for (uint16_t c=0; c<CAEN_DT7542_CH_NUM; c++){
            for (uint16_t i=0; i<CAEN_DT7542_SIGWIN; i++){
                acu+=evt.evt_buf[g][c][i];
                if (i==0xFF) bias = acu; //>>8
            } 
            avg =  acu - (bias<<2);
            //cout<<g<<c<<" bias="<<bias<<", acu="<<acu<<", avg="<<avg<<"; ";

            avgch.push_back(avg);
            acu = 0;
        }cout<<"\n";
    }
    acu=0;
    for (int32_t avgc: avgch)
        acu+=avgc;
    acu >>=12;
    cout<<std::to_string(acu)<<" ";

    Ne = acu;
    Te = Ne + rand() % 21 - 10;
}