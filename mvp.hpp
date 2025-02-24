/* Globus Thomson Scattering Digitizer
 * ITER Divertor Thomson Scattering (DTS 55.C4)
 * mvp.hpp
 * Model-View-Presenter
 * igor.bocharov@mail.ioffe.ru*/

#pragma once

#include <atomic>
#include <functional>

//View interface
class DigiView{
public:
    //name of view and stop flag
    DigiView(const string &name_):view_name(name_),stop(false){}

    //who am I
    string GetViewName(){return view_name;}

    //initialisation of view
    virtual void Init()=0;

    //runtime configuration of view
    virtual void Config(const string &param, const int val1, const int val2)=0;

    //start main cycle of the View
    virtual void MainLoop()=0;
    
    //stop main loop
    void StopMe(){stop=true;}   

    //view call presenter with "my_name, parameter = get_value"
    void SetPresenterCallback(std::function<void(string,string,int)> pres_callb_) {PresCallback=pres_callb_;}

    void CallPres(const string &param, const int val) {PresCallback(view_name, param, val);}
    
    //presenter call view with "parameter = set_value"
    //template<class T> void CallView(const string &param, const T)=0;
    virtual void CallView(const string &param, const int val)=0;
    virtual void CallView(const string &param, const string &val)=0;
    virtual void CallView(const string &param, const uint16_t val[])=0;

    //push string to log
    virtual void PushLog(const std::string& txt)=0;
    
protected:
    string view_name;       //view name string to inform presenter where from paramter incoming
    std::atomic<bool> stop; //flag to stop main cycle

private:
    std::function<void(string,string,int)> PresCallback; //calback to presenter
};

class DigiModel;
//A mediator between Views and model
class DigiPresenter{
public:
    DigiPresenter(DigiModel *model_):model(model_){threads.reserve(3);}
    
    //add new view object ptr
    void PushBack(DigiView *view){
        view->SetPresenterCallback( std::bind(&DigiPresenter::CallPresenter, this, 
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) );
        views.push_back(view);
    }

    void Init(){
        for(auto *v: views)
            v->Init();    
    }

    void Config(const string &param, const int val1, const int val2){
        for(auto *v: views)
            v->Config(param, val1, val2);    
    }

    //start thread with view object
    void StartMainLoop(){
        for(auto *v: views)
            threads.emplace_back( (v->MainLoop),v );  //==> thread!
    }

    //finish thread loop
    void StopMainLoop(){
        for(auto *v: views)
            v->StopMe();
    }

    //wait till all view threads stop
    void WaitMainLoop(){
        for(thread &t: threads)
            t.join();
    }

    //call each view to log
    void PushLog(const std::string& txt){
        for(DigiView *view: views)
            view->PushLog(txt);
    }
    
    template <class T>
    void CallView     (const string &view_, const string &param, const T val){
        //cout<<"pres->view "<<view_<<"-"<<param<<"-"<<val<<std::endl;
        for(DigiView *view: views)
            if (view->GetViewName()==view_)
                view->CallView(param,val);
    }

private:
    vector<DigiView *> views; //ptr to View objects
    vector<thread> threads;   //threads for Views
    DigiModel *model;         //model link
     
    //Event callback from View
    void CallPresenter(const string &caller, const string &param, const int val);
};

class DigiSet;
class Digitizer;

extern "C" {
#include "caen5742.h"
}
//Model interface
class DigiModel{
public:
    DigiModel():prr(this),curCoaxTab("BtnCoaxModeView"){} //wid_5742::tab_view

    virtual void DeviceMainLoop()=0;   //board asquisition loop

    void LoopStart();

    void f(){}

    void LoopStop(){
        prr.StopMainLoop();
        stop = false;
    }

    void LoopWait(){
        prr.WaitMainLoop();
        thr.join();
        }

    virtual void DeviceOpenToggle()=0;
    virtual void DeviceClose()=0;

    void ModelDeviceOpen (int state);

    void FillReg(); //fill view with setting parameters

    void ToggleReg(const string &sreg, const bool xml=false); //Toggle regisrer

    void FillRegCoax(); //fill tab of coax

    void ToggleRegCoax(const string &tname, const bool xml=false); //toggle tab of coax
    
    void MergeReg(); //merge settings and device state

    void UpdateState(); //update gui widgets with device state

    void AcqTrSet() {run_state.trg_in=1;} //SW trigger for led
    int  AsqIsBReady() {return run_state.board_rdy;} //is acq run

    void UpdateSignals(const caen5742_evt *evt);
    void DataCalc(const caen5742_evt *evt);
  
    void PushLog(const std::string& txt){prr.PushLog(txt);}

    DigiPresenter prr;
    
    string curCoaxTab; //BAD current active coax tab

protected:  
    void PushView(DigiView *dv){ prr.PushBack(dv);}  //add view
    DigiSet *sets;           //ptr to device settings
    std::atomic<bool> stop;  //device thread flag
    acq_stat run_state;

private:
    thread thr;   // Board loop =>
};

/*class ViewGui: public DigiView{
    public:
    using DigiView::DigiView;
    void Init(){}
    void MainLoop(){
        while(!stop){
            std::this_thread::sleep_for(std::chrono::seconds(1));
            CallPres(" govno", 123);
        }
    }
    void CallView(const string &param, const int val){
        cout<<view_name<<param<<val<<std::endl;
    }
    void PushLog(const std::string& txt) {cout<<"gui "<<txt<<std::endl;}
};

class ViewCon: public DigiView{
    public:
    using DigiView::DigiView;
    void Init(){cout<<"init con\n";}
    void MainLoop(){
        while(!stop){
            std::this_thread::sleep_for(std::chrono::milliseconds(750));
            CallPres(" ochko ", 456);
        }
    }
    void CallView(const string &param, const int val){
    }
void PushLog(const std::string& txt) {cout<<"con "<<txt<<std::endl;}
};*/