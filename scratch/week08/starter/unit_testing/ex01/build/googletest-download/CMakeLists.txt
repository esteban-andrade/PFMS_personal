cmake_minimum_required(VERSION 2.8.2)

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(googletest
  GIT_REPOSITORY    https://github.com/google/googletest.git
  GIT_TAG           master
  SOURCE_DIR        "/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-src"
  BINARY_DIR        "/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)
