#include <openrave-core.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace OpenRAVE;

class TestBase
{
  public:
    TestBase(string ,string,bool showgui=true);
    ~TestBase();
    void SetCamera(dReal q0, dReal q1, dReal q2, dReal q3, dReal tx, dReal ty, dReal tz);

  protected:
    EnvironmentBasePtr penv;
    ViewerBasePtr viewer;
    boost::thread *pthviewer;
    RobotBasePtr probot;
    ControllerBasePtr pcontroller;
    bool showgui;

  private:
    void SetViewer();

};


