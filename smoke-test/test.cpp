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

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/pids.h>

using namespace std;
using namespace robottestingframework;
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
class TestAssignmentSimpleControlDesign : public yarp::robottestingframework::TestCase
{
    PolyDriver driver;
    IEncoders *ienc;

    BufferedPort<Bottle> portL;
    BufferedPort<Bottle> portR;
    RpcClient portBall;

    Integrator I;

    /******************************************************************/
    bool setBall(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addVocab32("set");
            cmd.addFloat64(pos[0]);
            cmd.addFloat64(pos[1]);
            cmd.addFloat64(pos[2]);
            if (!portBall.write(cmd,reply))
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to world");
            if (reply.get(0).asVocab32()!=Vocab32::encode("ack"))
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Invalid reply from world");
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
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Met requirements on \"%s\" => gained %d points",
                                             s.getName().c_str(),points));
            score+=points;
            reached=true;
        }
    }

public:
    /******************************************************************/
    TestAssignmentSimpleControlDesign() :
        yarp::robottestingframework::TestCase("TestAssignmentSimpleControlDesign"),
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
        float rpcTmo=(float)property.check("rpc-timeout",Value(10.0)).asFloat64();

        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icubSim/head");
        option.put("local","/"+getName());

        if (!driver.open(option))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to icubSim");
        driver.view(ienc);

        portL.open("/"+getName()+"/target/left:i");
        if (!Network::connect("/left/detector/target",portL.getName()))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to left target");

        portR.open("/"+getName()+"/target/right:i");
        if (!Network::connect("/right/detector/target",portR.getName()))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to right target");

        string portBallName("/"+getName()+"/ball:rpc");
        portBall.open(portBallName);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        portBall.asPort().setTimeout(rpcTmo);
        if (!Network::connect(portBallName,"/assignment_control-pid-ball/rpc"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /assignment_control-pid-ball/rpc");

        Rand::init();
        Time::useNetworkClock("/clock");
        return true;
    }

    /******************************************************************/
    void tearDown() override
    {
        Time::useSystemClock();
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
        x0[0]=-1.0;
        x0[1]=0.0;
        x0[2]=1.0;
        setBall(x0);

        Time::delay(5.0);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Checking controller against p2p movements");
        double R=0.2;
        double theta=(M_PI/180.0)*(Rand::scalar(-20.0,20.0)+
                                   180.0*round(Rand::scalar(0.0,1.0)));
        Vector dx(3,0.0);
        dx[1]=R*cos(theta);
        dx[2]=R*sin(theta);
        Vector x=x0+dx;
        setBall(x);

        Vector c(2);
        c[0]=320/2; c[1]=240/2;

        int nAxes; ienc->getAxes(&nAxes);
        vector<double> encs(nAxes);

        unsigned int N=20;
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
                ul.push(b->get(1).asInt32());
                vl.push(b->get(2).asInt32());
            }

            if (Bottle *b=portR.read(false))
            {
                ur.push(b->get(1).asInt32());
                vr.push(b->get(2).asInt32());
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

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Checking controller against tracking");
        Statistics track_ul("ul",N,c[0],6,3),track_vl("vl",N,c[1],6,3);
        reached.fill(false);

        double Ts=0.02;
        I.setTs(Ts);
        I.reset(x);
        Vector v=x0-x;
        v*=0.1/norm(v);

        t0=Time::now();
        while (true)
        {
            double t=Time::now()-t0;
            setBall(I.integrate(v));

            if (Bottle *b=portL.read(false))
            {
                track_ul.push(b->get(1).asInt32());
                track_vl.push(b->get(2).asInt32());
            }

            assign_points(track_ul,reached[0],8,score);
            assign_points(track_vl,reached[1],4,score);

            if ((t>5.0) || (reached[0]&&reached[1]))
                break;

            Time::delay(Ts);
        }

        score=std::min(score,100U);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score>0,Asserter::format("Total score = %d",score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentSimpleControlDesign)
