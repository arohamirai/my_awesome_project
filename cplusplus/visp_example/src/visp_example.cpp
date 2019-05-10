#include <boost/math/special_functions/sign.hpp>
#include <unistd.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
int main(int argc, char** argv)
{
  vpPlot graph(3, 800, 800, 400, 10, "Curves...");
  graph.initGraph(0, 2);
  graph.initGraph(1, 2);
  graph.initGraph(2, 1);
  graph.initRange(0, -1, 1, -0.2, 0.2);
  graph.initRange(1, -1, 1, -1, 0);
  graph.initRange(2, -1, 1, -0.05, 0.05);
  graph.setTitle(0, "Velocities");
  graph.setTitle(1, "Error s-s*");
  graph.setTitle(2, "Trajectory");
  graph.setLegend(0, 0, "vx");
  graph.setLegend(0, 1, "wz");
  graph.setLegend(1, 0, "ex");
  graph.setLegend(1, 1, "ey");
  graph.setLegend(2, 0, "Trajectory");

  vpHomogeneousMatrix wMc, wMo, cMo, oMc;
  vpSimulatorCamera robot;
  robot.setSamplingTime(0.02);           // Modify the default sampling time to 0.1 second
  robot.setMaxTranslationVelocity(0.2);  // vx, vy and vz max set to 1 m/s
  robot.setMaxRotationVelocity(0.5);     // wx, wy and wz max set to 90 deg/s

  vpThetaUVector thetaU;
  vpTranslationVector t;

  double oMc_x = 1.;
  double oMc_y = atof(argv[1]);
  std::cout << oMc_y << std::endl;
  double adjust_angle =
    vpMath::rad(50. * boost::math::sign(oMc_x) * oMc_y);
  std::cout << "adjust_angle: " << adjust_angle * 180/ M_PI <<"degree."  << std::endl;
  getchar();


  thetaU.buildFrom(0, 0, adjust_angle);
  t.buildFrom(0, oMc_y, 0);
  wMc.buildFrom(t, thetaU);

  thetaU.buildFrom(0, 0, 0);
  t.buildFrom(-oMc_x, 0, 0);
  wMo.buildFrom(t, thetaU);

  robot.setPosition(wMc);

  double last_vx = 0;
  double vx, wz;
  double theta;

  for (int i = 0; i < 500; i++)
  {
    robot.getPosition(wMc);
    // std::cout << "wMc:\n" << wMc << std::endl;
    cMo = wMc.inverse() * wMo;
    oMc = cMo.inverse();
    thetaU.buildFrom(oMc);
    theta = std::atan2(oMc[1][0], oMc[0][0]);

    vx = 4. * cMo[0][3];
    wz = -30 * fabs(last_vx) * theta - 120 * (last_vx * sin(theta) / theta * oMc[1][3]);

    vpColVector v(6), vv(2), error(2);
    v[0] = vx;
    v[5] = wz;
    last_vx = vx;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);

    vv[0] = vx;
    vv[1] = wz;
    error[0] = oMc[0][3];
    error[1] = oMc[1][3];

    graph.plot(0, i, vv);
    graph.plot(1, i, error);
    graph.plot(2, 0, oMc[0][3], oMc[1][3]);
    // graph.plot(1, i, task.getError());
    // getchar();
    usleep(1000 * 33);
  }

  const char* legend = "Click to quit...";
  vpDisplay::displayText(graph.I, ( int )graph.I.getHeight() - 60, ( int )graph.I.getWidth() - 150,
                         legend, vpColor::red);
  vpDisplay::flush(graph.I);
  vpDisplay::getClick(graph.I);
}
