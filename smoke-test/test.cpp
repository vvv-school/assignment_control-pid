/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <vector>
#include <deque>
#include <cmath>
#include <array>
#include <algorithm>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include <yarp/rtf/TestCase.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/pids.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/**********************************************************************/
class Statistics
{
    deque<double> buf;
    string name;
    unsigned int N;
    double M,T,S;

public:
    /******************************************************************/
    Statistics(const string &name_, const unsigned int N_,
               const double M_, const double T_, const double S_) :
               name(name_), N(N_), M(M_), T(T_), S(S_) { }

    /******************************************************************/
    string getName() const { return name; }

    /******************************************************************/
    void reset() { buf.clear(); }

    /******************************************************************/
    void push(const double v)
    {
        buf.push_back(v);
        if (buf.size()>N)
            buf.pop_front();
    }

    /******************************************************************/
    bool reached() const
    {
        if (buf.size()==N)
        {
            double mean=0.0;
            double stdev=0.0;

            for (auto d:buf)
            {
                mean+=d;
                stdev+=d*d;
            }

            mean/=N;
            stdev=sqrt(stdev/N-mean*mean);
            return ((fabs(M-mean)<T) && (stdev<S));
        }
        else
            return false;
    }
};

/**********************************************************************/
class TestAssignmentSimpleControlDesign : public yarp::rtf::TestCase
{
    PolyDriver driver;
    IEncoders *ienc;

    BufferedPort<Bottle> portL;
    BufferedPort<Bottle> portR;
    RpcClient portBall;

    Integrator I;

    /******************************************************************/
    bool createBall(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addString("world");
            cmd.addString("mk");
            cmd.addString("ssph");
            cmd.addDouble(0.03);
            cmd.addDouble(pos[0]);
            cmd.addDouble(pos[1]);
            cmd.addDouble(pos[2]);
            cmd.addDouble(1.0);
            cmd.addDouble(0.0);
            cmd.addDouble(0.0);
            RTF_ASSERT_ERROR_IF(portBall.write(cmd,reply),
                                "Unable to talk to world");
            return true;
        }
        else
            return false;
    }

    /******************************************************************/
    bool setBall(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addString("world");
            cmd.addString("set");
            cmd.addString("ssph");
            cmd.addInt(1);
            cmd.addDouble(pos[0]);
            cmd.addDouble(pos[1]);
            cmd.addDouble(pos[2]);
            RTF_ASSERT_ERROR_IF_FALSE(portBall.write(cmd,reply),
                                      "Unable to talk to world");
            return true;
        }
        else
            return false;
    }

    /******************************************************************/
    void assign_points(const Statistics &s, bool &reached,
                       const unsigned int points, unsigned int &score)
    {
        if (s.reached() && !reached)
        {
            RTF_TEST_REPORT(Asserter::format("Met requirements on \"%s\" => gained %d points",
                                             s.getName().c_str(),points));
            score+=points;
            reached=true;
        }
    }

public:
    /******************************************************************/
    TestAssignmentSimpleControlDesign() :
        yarp::rtf::TestCase("TestAssignmentSimpleControlDesign"),
        I(0.0,Vector(3,0.0))
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentSimpleControlDesign()
    {
    }

    /******************************************************************/
    bool setup(yarp::os::Property& property) override
    {
        float rpcTmo=(float)property.check("rpc-timeout",Value(10.0)).asDouble();

        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icubSim/head");
        option.put("local","/"+getName());

        RTF_ASSERT_ERROR_IF_FALSE(driver.open(option),"Unable to connect to icubSim");
        driver.view(ienc);

        portL.open("/"+getName()+"/target/left:i");
        RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/left/detector/target",
                                                   portL.getName()),
                                  "Unable to connect to left target");

        portR.open("/"+getName()+"/target/right:i");
        RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/right/detector/target",
                                                   portR.getName()),
                                  "Unable to connect to right target");

        string portBallName("/"+getName()+"/ball:rpc");
        portBall.open(portBallName);
        RTF_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        portBall.asPort().setTimeout(rpcTmo);
        RTF_ASSERT_ERROR_IF_FALSE(Network::connect(portBallName,"/icubSim/world"),
                                  "Unable to connect to /icubSim/world");

        Rand::init();

        return true;
    }

    /******************************************************************/
    void tearDown() override
    {
        portL.close();
        portR.close();
        portBall.close();
        driver.close();
    }

    /******************************************************************/
    void run() override
    {
        unsigned int score=0;

        Vector x0(3);
        x0[0]=0.0;
        x0[1]=1.0;
        x0[2]=0.75;
        createBall(x0);

        // connect detectors to controller only when the ball is in the world
        RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/left/detector/target",
                                                   "/controller/target/left:i"),
                                  "Unable to connect left detector to controller");

        RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/right/detector/target",
                                                   "/controller/target/right:i"),
                                  "Unable to connect right detector to controller");

        Time::delay(5.0);

        RTF_TEST_REPORT("Checking controller against p2p movements");
        double R=0.2;
        double theta=(M_PI/180.0)*(Rand::scalar(-20.0,20.0)+
                                   180.0*round(Rand::scalar(0.0,1.0)));
        Vector dx(3,0.0);
        dx[0]=R*cos(theta);
        dx[1]=R*sin(theta);
        Vector x=x0+dx;
        setBall(x);

        Vector c(2);
        c[0]=320/2; c[1]=240/2;

        int nAxes; ienc->getAxes(&nAxes);
        vector<double> encs(nAxes);

        unsigned int N=10;
        Statistics ul("ul",N,c[0],4,2),vl("vl",N,c[1],4,2),
                   ur("ur",N,c[0],4,2),vr("vr",N,c[1],4,2),
                   tilt("tilt",N,0,2,1),pan("pan",N,0,2,1);

        array<bool,6> reached;
        reached.fill(false);

        double t0=Time::now();
        while (true)
        {
            double t=Time::now()-t0;
            if (Bottle *b=portL.read(false))
            {
                ul.push(b->get(0).asInt());
                vl.push(b->get(1).asInt());
            }

            if (Bottle *b=portR.read(false))
            {
                ur.push(b->get(0).asInt());
                vr.push(b->get(1).asInt());
            }

            ienc->getEncoders(encs.data());
            tilt.push(encs[3]);
            pan.push(encs[4]);

            assign_points(ul,reached[0],4,score);
            assign_points(vl,reached[1],1,score);
            assign_points(ur,reached[2],4,score);
            assign_points(vr,reached[3],1,score);

            if (reached[0]||reached[2])
                assign_points(tilt,reached[4],2,score);
            if (reached[1]||reached[3])
                assign_points(pan,reached[5],2,score);

            bool cumul=true;
            for (auto b:reached)
                cumul&=b;

            if ((t>10.0) || cumul)
                break;

            Time::delay(0.1);
        }

        RTF_TEST_REPORT("Checking controller against tracking");
        ul.reset(); vl.reset();
        reached.fill(false);

        double Ts=0.02;
        I.setTs(Ts);
        I.reset(x);
        Vector v=x0-x;
        v*=0.15/norm(v);

        t0=Time::now();
        while (true)
        {
            double t=Time::now()-t0;
            setBall(I.integrate(v));

            if (Bottle *b=portL.read(false))
            {
                ul.push(b->get(0).asInt());
                vl.push(b->get(1).asInt());
            }

            assign_points(ul,reached[0],8,score);
            assign_points(vl,reached[1],4,score);

            if ((t>5.0) || (reached[0]&&reached[1]))
                break;

            Time::delay(Ts);
        }

        score=std::min(score,100U);
        RTF_TEST_CHECK(score>0,Asserter::format("Total score = %d",score));
    }
};

PREPARE_PLUGIN(TestAssignmentSimpleControlDesign)
