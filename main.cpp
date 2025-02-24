/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * main.cpp
 * entry point */

#include "digitizer.hpp"

const string def_fconf="dt5742.xml";

int main(int argc, char** argv) {
    cout<<"GlobusTS"<<std::endl;
        string fconf(def_fconf);
        if (argv[1]) fconf=string(argv[1]);

    Digitizer dgtz;
        dgtz.LoadConf(fconf);
        dgtz.Run();
  
    return 0;
}

//TEST
extern "C" void test_caen_speed();

int main2(int argc, char** argv) {
    test_caen_speed();
    return 0;
}

//EOF