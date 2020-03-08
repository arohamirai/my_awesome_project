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
#include <memory>
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

  std::shared_ptr<vpServo> p_task_x_y_;
  vpSimulatorCamera robot;
  robot.setSamplingTime(
    0.02);  // Modify the default sampling time to 0.1 second
  robot.setMaxTranslationVelocity(0.2);  // vx, vy and vz max set to 1 m/s
  robot.setMaxRotationVelocity(0.5);     // wx, wy and wz max set to 90 deg/s

  vpVelocityTwistMatrix cVe_eye_;
  vpMatrix eJe_wz_, eJe_vx_wz_;

  vpFeaturePoint3D feature_x_y_;
  vpFeaturePoint3D feature_x_y_d_;

  cVe_eye_.eye();
  eJe_vx_wz_.resize(6, 2);
  eJe_vx_wz_ = 0;
  eJe_vx_wz_[0][0] = 1;  // vx
  eJe_vx_wz_[5][1] = 1;  // wz

  p_task_x_y_ = std::make_shared<vpServo>();
  p_task_x_y_->setServo(vpServo::EYEINHAND_L_cVe_eJe);
  p_task_x_y_->setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
  p_task_x_y_->setLambda(1.0);
  p_task_x_y_->set_cVe(cVe_eye_);
  p_task_x_y_->set_eJe(eJe_vx_wz_);

  vpTranslationVector t;
  vpRzyxVector r;
  vpHomogeneousMatrix wMc, wMcd;

  t.buildFrom(-0.1, -0.1, 1);
  r.buildFrom(M_PI / 6, 0, 0);
  wMc.buildFrom(t, vpRotationMatrix(r));
  wMcd.eye();

  feature_x_y_.buildFrom(wMc[0][3], wMc[1][3], 1);
  feature_x_y_d_.buildFrom(wMcd[0][3], wMcd[1][3], 1);
  p_task_x_y_->addFeature(feature_x_y_, feature_x_y_d_,
                          vpFeaturePoint3D::selectX()
                            | vpFeaturePoint3D::selectY());

  robot.setPosition(wMc);

  for (int i = 0; i < 500; i++)
  {
    robot.getPosition(wMc);
    // std::cout << "wMc:\n" << wMc << std::endl;
    vpHomogeneousMatrix cMcd = wMc.inverse() * wMcd;
    feature_x_y_.buildFrom(cMcd[0][3], cMcd[1][3], 1);

    vpColVector v = p_task_x_y_->computeControlLaw();
    vpColVector error = p_task_x_y_->getError();

    vpColVector vv(6);
    vv[0] = v[0];
    vv[5] = v[1];
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, vv);


    graph.plot(0, i, v);
    graph.plot(1, i, error);
    // graph.plot(1, i, task.getError());
    // getchar();
    // usleep(1000 * 33);
  }

  getchar();

  const char* legend = "Click to quit...";
  vpDisplay::displayText(graph.I, ( int )graph.I.getHeight() - 60,
                         ( int )graph.I.getWidth() - 150, legend, vpColor::red);
  vpDisplay::flush(graph.I);
  vpDisplay::getClick(graph.I);
}
