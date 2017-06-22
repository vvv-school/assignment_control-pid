// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Assignment on how to design a simple PID controller.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>


#include <cmath>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/pids.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class Mover: public RFModule
{
private:
    RpcClient port;
    Vector x0,v;
        
    enum class State { init, p2p, track };
    State state;
    Integrator I;
    
    double t0;
    int cnt;
    
    void createBall()
    {
        Bottle cmd,reply;
        cmd.addString("world");
        cmd.addString("mk");
        cmd.addString("ssph");
        cmd.addDouble(0.03);
        cmd.addDouble(x0[0]);
        cmd.addDouble(x0[1]);
        cmd.addDouble(x0[2]);
        cmd.addDouble(1.0);
        cmd.addDouble(0.0);
        cmd.addDouble(0.0);
        port.write(cmd,reply);        
    }
    
    bool setBall(const Vector &x)
    {
        if (x.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addString("world");
            cmd.addString("set");
            cmd.addString("ssph");
            cmd.addInt(1);
            cmd.addDouble(x[0]);
            cmd.addDouble(x[1]);
            cmd.addDouble(x[2]);
            port.write(cmd,reply);
            return true;
        }
        else
            return false;
    }
    
    Vector draw_point()
    {
        double R=0.2;
        double theta=(M_PI/180.0)*(Rand::scalar(-20.0,20.0)+
                                   180.0*round(Rand::scalar(0.0,1.0)));
        Vector dx(3,0.0);
        dx[0]=R*cos(theta);
        dx[1]=R*sin(theta);
        return dx;        
    }
    
    void draw_velocity(const Vector &x)
    {
        v=x0-x;
        double n=norm(v);
        if (n>0.0)
            v*=0.075/norm(v); // [m/s]

        I.reset(x);
        t0=Time::now();
    }

public:
    Mover() : state(State::init), I(0.0,Vector(3,0.0)) { }
    
    virtual bool configure(ResourceFinder &rf)
    {
        port.open("/mover");
        if (!Network::connect(port.getName().c_str(),"/icubSim/world"))
        {
            yError()<<"Unable to connect to the world!";
            port.close();
            return false;
        }

        x0.resize(3);
        x0[0]=0.0;
        x0[1]=1.0;
        x0[2]=0.75;

        Rand::init();
        I.setTs(getPeriod());
        
        return true;
    }

    virtual bool close()
    {
        port.close();
        return true;
    }

    virtual double getPeriod()
    {
        return 0.02;
    }

    virtual bool updateModule()
    {
        if (state==State::init)
        {
            createBall();            
            Time::delay(5.0);
            state=State::p2p;
            cnt=0;
        }
        else if (state==State::p2p)
        {
            Vector x=x0+draw_point();
            setBall(x);
            Time::delay(15.0);
            if (++cnt>2)
            {
                draw_velocity(x);
                state=State::track;
                cnt=0;
            }
        }
        else
        {
            Vector x=I.integrate(v);
            setBall(x);
            if (Time::now()-t0>10.0)
            {
                draw_velocity(x);
                if (++cnt>2)
                {
                    state=State::p2p;
                    cnt=0;
                }
            }
        }
        
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
    rf.configure(argc,argv);

    Mover mover;
    return mover.runModule(rf);
}
