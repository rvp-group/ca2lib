#include <ros/ros.h>

#include <QApplication>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "ca2lib/gui/ca2lib_application.h"
#include "ca2lib/gui/image_frame.h"
#include "ca2lib/gui/ros_image_view.h"
#include "ca2lib/io/ros.h"
#include "ca2lib/plane_extractor.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "qt_example");
  QApplication app(argc, argv);

  ca2lib::Ca2libApplication window("/image_in", "/cloud_in");

  std::thread thrd_ros([]() {
    std::cerr << "Spinning shit" << std::endl;
    ros::spin();
  });

  window.show();

  return app.exec();
}