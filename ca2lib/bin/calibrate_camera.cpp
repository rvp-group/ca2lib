#include <filesystem>
#include <opencv2/highgui.hpp>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/targets.h"

using namespace srrg2_core;

const char* whatdoes =
    "Calibrate a single camera given a set of input images and the "
    "corresponding calibration target";

int main(int argc, char** argv) {
  ParseCommandLine parser(argv, &whatdoes);

  ArgumentString input_dir(
      &parser, "i", "input-dir",
      "Directory containing calibration images for the camera", "");
  ArgumentString output_f(&parser, "o", "output",
                          "Output calibration file [ends for .yaml or .json]",
                          "calibration_result.yaml");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");

  parser.parse();

  if (!input_dir.isSet() or !target_f.isSet()) {
    std::cerr << "No input folder or target specified.\n"
              << parser.options() << std::endl;
    return -1;
  }

  return 0;
}