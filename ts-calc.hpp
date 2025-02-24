/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * ts-calc.hpp
 * Thomson scattering calculator
 * igor.bocharov@mail.ioffe.ru*/

#pragma include once

class TSalgo{
public:
    TSalgo(const caen5742_evt& evt);
    void Calc();

    int Te;
    int Ne;
private:
    const caen5742_evt& evt;
};