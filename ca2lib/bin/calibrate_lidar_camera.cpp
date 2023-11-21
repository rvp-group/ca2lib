#include "ca2lib/parse_command_line.h"

using namespace srrg2_core;

const char* whatdoes = "Calibrate LiDAR-RGB Camera pair.";

int main(int argc, char** argv) {
  ParseCommandLine parser(argv, &whatdoes);

  return 0;
}