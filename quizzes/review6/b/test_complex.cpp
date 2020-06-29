#include "gtest/gtest.h"
#include <iostream>
#include "complex.h"

//The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST (ComplexNumTest, Divide) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { 2.5, 0.5 };
  TComplex answer = Complex::divide(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
