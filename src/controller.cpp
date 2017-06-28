// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Assignment on how to design a simple PID controller.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <array>
#include <string>
#include <fstream>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


// The class implementing our PID control
class PID
{
    double Ts,Kp,Ki;

public:
    // constructor
    PID() : Ts(0.0), Kp(0.0), Ki(0.0) { }
        
    // helper function to set up sample time and gains
    void set(const double Ts, const double Kp, const double Ki)
    {
        this->Ts=Ts;
        this->Kp=Kp;
        this->Ki=Ki;
    }
    
    // compute the control command
    double command(const double reference, const double feedback)
    {
        // FILL IN THE CODE
        return 0.0;
    }
};


template<typename T>
string toString(const vector<T> &in)
{
    string str;
    for (auto v:in)
        str+=to_string(v)+" ";
    return str;
}

template<typename T>
Bottle toBottle(const vector<T> &in)
{    
    Bottle b;
    for (auto v:in)
        b.addDouble(v);
    return b;
}


class CtrlThread: public RateThread
{
protected:
    PolyDriver         driver;
    IControlMode2     *imod;
    IEncoders         *ienc;
    IVelocityControl2 *ivel;

    BufferedPort<Bottle> portL;
    BufferedPort<Bottle> portR;
    BufferedPort<Bottle> portScope;
    
    ResourceFinder *rf;
    ofstream fout;
    double t0;
    
    array<PID,1> controllers;
    int nAxes;
    int ul,vl;
    int ur,vr;

public:
    CtrlThread() : RateThread(1000), rf(nullptr) { }
    
    void setRF(ResourceFinder &rf)
    {
        this->rf=&rf;
    }

    virtual bool threadInit()
    {
        // open the device driver to control the head
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icubSim/head");
        option.put("local","/controller");

        if (!driver.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        // open the views
        driver.view(imod);
        driver.view(ienc);
        driver.view(ivel);
        
        // tell the device we aim to control
        // in velocity mode all the joints
        ienc->getAxes(&nAxes);
        vector<int> modes(nAxes,VOCAB_CM_VELOCITY);
        imod->setControlModes(modes.data());

        // open ports
        portL.open("/controller/target/left:i");
        portR.open("/controller/target/right:i");
        portScope.open("/controller/scope:o");

        // set up our controllers
        double Ts=getRate()*0.001;  // period is given in [ms]
        controllers[0].set(Ts,0.0,0.0);
        
        // initialize the pixels == image center
        ul=ur=320/2;
        vl=vr=240/2;
        
        // open file for datalog
        if (rf)
            fout.open((rf->getHomeContextPath()+"/out.txt").c_str());
        else
            fout.open("out.txt");
            
        t0=Time::now();
        
        return true;
    }

    virtual void run()
    {
        // poll data from YARP network
        // "false" means non-blocking read
        Bottle *pTargetL=portL.read(false);
        Bottle *pTargetR=portR.read(false);
        
        // update local copies if
        // something has arrived
        if (pTargetL!=nullptr)
        {
            ul=pTargetL->get(0).asInt();
            vl=pTargetL->get(1).asInt();
        }

        if (pTargetR!=nullptr)
        {
            ur=pTargetR->get(0).asInt();
            vr=pTargetR->get(1).asInt();
        }
        
        // get current joint encoders
        vector<double> encs(nAxes);
        ienc->getEncoders(encs.data());
        
        // perform one control instance
        double eyes_pan=controllers[0].command(0.0,0.0);
        
        // send commands to the robot head
        vector<double> velocity(nAxes);
        velocity[0]=0.0;                // neck pitch
        velocity[1]=0.0;                // neck roll
        velocity[2]=0.0;                // neck yaw
        velocity[3]=0.0;                // eyes tilt
        velocity[4]=eyes_pan;           // eyes pan
        velocity[5]=0.0;                // eyes vergence
        ivel->velocityMove(velocity.data());
        
        // datalog
        vector<int> pixels(4);
        pixels[0]=ul; pixels[1]=vl;
        pixels[2]=ur; pixels[3]=vr;
        
        string log=toString(pixels)+toString(encs)+toString(velocity)+"\n";
        
        // provide information to yarplogger
        yInfo()<<log;
        
        // save information on file to be plotted with Octave
        fout<<to_string(Time::now()-t0)+" "+log;
        
        // send information to yarpscope
        Bottle &scope=portScope.prepare();
        scope=toBottle(pixels);
        scope.append(toBottle(encs));
        scope.append(toBottle(velocity));
        portScope.write();
    }

    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the driver for safety reason
        ivel->stop();
        
        fout.close();
                
        portL.close();
        portR.close();
        portScope.close();
        
        driver.close();
    }
};


class CtrlModule: public RFModule
{
protected:
    CtrlThread thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // retrieve command line options in the form of "--period 0.01"
        double period=rf.check("period",Value(0.01)).asDouble();

        // set the thread rate: integer accounting for [ms]
        thr.setRate(int(period*1000.0));
        
        // pass on the resource finder
        thr.setRF(rf);

        return thr.start();
    }

    virtual bool close()
    {
        thr.stop();
        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.setDefaultContext("assignment_control-pid");
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
