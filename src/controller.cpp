// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Assignment on how to design a simple PID controller.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <array>
#include <string>
#include <fstream>
#include <vector>
#include <limits>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IVelocityControl.h>
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


class CtrlThread: public PeriodicThread
{
protected:
    PolyDriver        driver;
    IControlMode     *imod;
    IEncoders        *ienc;
    IVelocityControl *ivel;

    BufferedPort<Bottle> portL;
    BufferedPort<Bottle> portR;
    BufferedPort<Bottle> portScope;

    ResourceFinder *rf;
    ofstream fout;
    double t0;

    enum class ControlState { idle, half_armed, armed, run };
    ControlState state;

    array<PID,1> controllers;   // FILL IN THE CODE
    int nAxes;
    bool objl,objr;
    int ul,vl;
    int ur,vr;

    void handleControlState()
    {
        if (state==ControlState::idle)
            state=ControlState::half_armed;
        else if (state==ControlState::half_armed)
            state=ControlState::armed;
    }

public:
    CtrlThread() : PeriodicThread(1.0), rf(nullptr) { }

    void setRF(ResourceFinder &rf)
    {
        this->rf=&rf;
    }

    bool threadInit() override
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

        // retrieve number of axes
        ienc->getAxes(&nAxes);

        // open ports
        portL.open("/controller/target/left:i");
        portR.open("/controller/target/right:i");
        portScope.open("/controller/scope:o");

        // set up our controllers
        double Ts=getPeriod();  // period is given in [s]
        // FILL IN THE CODE
        controllers[0].set(Ts,0.0,0.0);
        
        // initialize the pixels == image center
        objl=objr=false;
        ul=ur=320/2;
        vl=vr=240/2;

        // open file for datalog
        if (rf!=nullptr)
            fout.open((rf->getHomeContextPath()+"/out.txt").c_str());
        else
            fout.open("out.txt");

        state=ControlState::idle;
        t0=Time::now();

        return true;
    }

    void run() override
    {
        // poll data from YARP network
        // "false" means non-blocking read
        Bottle *pTargetL=portL.read(false);
        Bottle *pTargetR=portR.read(false);

        // update local copies if
        // something has arrived
        if (pTargetL!=nullptr)
        {
            objl=(pTargetL->get(0).asInt()>0);
            ul=pTargetL->get(1).asInt();
            vl=pTargetL->get(2).asInt();
            handleControlState();
        }

        if (pTargetR!=nullptr)
        {
            objr=(pTargetR->get(0).asInt()>0);
            ur=pTargetR->get(1).asInt();
            vr=pTargetR->get(2).asInt();
            handleControlState();
        }

        // tell the device we aim to control
        // in velocity mode all the joints
        if (state==ControlState::armed)
        {
            vector<int> modes(nAxes,VOCAB_CM_VELOCITY);
            imod->setControlModes(modes.data());

            // disable slew rate on velocity commands
            // that are due to limited accelerations
            vector<double> accs(nAxes,numeric_limits<double>::max());
            ivel->setRefAccelerations(accs.data());
            state=ControlState::run;
        }

        // get current joint encoders
        vector<double> encs(nAxes);
        ienc->getEncoders(encs.data());

        // velocity to be sent to the robot
        vector<double> velocity(nAxes,0.0);

        // perform one control instance
        if ((state==ControlState::run) && objl && objr)
        {
            // FILL IN THE CODE
            double eyes_pan=controllers[0].command(0.0,0.0);
        
            // send commands to the robot head
            velocity[0]=0.0;                // neck pitch
            velocity[1]=0.0;                // neck roll
            velocity[2]=0.0;                // neck yaw
            velocity[3]=0.0;                // eyes tilt
            velocity[4]=eyes_pan;           // eyes pan
            velocity[5]=0.0;                // eyes vergence
            ivel->velocityMove(velocity.data());
        }

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

    void threadRelease() override
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
    bool configure(ResourceFinder &rf) override
    {
        // retrieve command line options in the form of "--period 0.01"
        double period=rf.check("period",Value(0.01)).asDouble();

        // set the thread period in [s]
        thr.setPeriod(period);

        // pass on the resource finder
        thr.setRF(rf);

        return thr.start();
    }

    bool close() override
    {
        thr.stop();
        return true;
    }

    double getPeriod() override
    {
        return 1.0;
    }

    bool updateModule() override
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
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("assignment_control-pid");
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
