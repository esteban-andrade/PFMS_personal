#include "gtest/gtest.h"
#include <iostream>
#include "complex.h"

//The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST(ComplexNumTest, Divide)
{
  //Two different methods of initialising a struct
  TComplex a = {.re = 2, .im = 3};
  TComplex b = {1, 1};
  TComplex expected = {2.5, 0.5};
  TComplex answer = Complex::divide(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, magniture)
{
  TComplex a = {.re = 4, .im = 3};
  double expected = {5};
  double answer = Complex::magnitude(a);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected, answer);
}
TEST(ComplexNumTest, conjugate)
{
  TComplex a = {.re = 4, .im = 3};
  TComplex expected = {4, -3};
  TComplex answer = Complex::conjugate(a);
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}
TEST(ComplexNumTest, add)
{
  TComplex a = {4, -3};
  TComplex b = {1, 1};
  TComplex expected = {5, -2};
  TComplex answer = Complex::add(a, b);
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}
TEST(ComplexNumTest, subtract)
{
  TComplex a = {4, -3};
  TComplex b = {1, 1};
  TComplex expected = {3, -4};
  TComplex answer = Complex::subtract(a, b);
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}
TEST(ComplexNumTest, multiply)
{
  TComplex a = {4, -3};
  TComplex b = {1, 1};
  TComplex expected = {7, 1};
  TComplex answer = Complex::multiply(a, b);
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}
TEST(ComplexNumTest, format)
{
  TComplex a = {.re = -4, .im = -3};
  std::stringstream expected;
  expected << a.re << " + " << a.im << "i";
  std::string answer = Complex::format(a);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
