#include <iostream>
#include <vector>
#include <chrono>
#include <random>


using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double> &numbers);

void printVec(vector<double> &num)
{
  for(auto i : num)
    {
      cout << i <<endl;
    }
  cout << endl;
}

int main () {
  //TODO Create a vector of doubles with 4 values

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937 gen(seed);
  std::uniform_int_distribution<std::mt19937::result_type> dist(0,100);

  vector<double> vec;
  for(int i = 0; i < 4; i++)
    {
      vec.push_back(dist(gen));
    }
  cout << "Create a vector of doubles with 4 values" << endl;
  printVec(vec);
  //TODO Add a value to the front.
  vec.insert(vec.begin(),dist(gen));
  cout << "Add a value to the front" << endl;
  printVec(vec);

  //TODO Modify the 3rd value
  cout << "Modify the 3rd Value" << endl;
  vec[2] =  dist(gen);

  printVec(vec);

  //TODO Print out the numbers
  // Using a Range-based for loop with the auto keyword
  //TODO Compute the sum via sun function and print sum
  cout << "The sum of this vector is : " << sum(vec) << endl;
  return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers) {
  double total = 0.0;
  //TODO Use an iterator
  auto it = numbers.begin();
  total = std::accumulate(it,it+numbers.size(), total );
  return total;
}
