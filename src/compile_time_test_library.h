// This code allows you to emit all the values of a const int[] during compilation
// If a compile time test fails, you can use this as a diagnostic to see all the values.
//
// This code borrows from a bunch of different ideas on Stack Overflow.
// The idea, working backwards, is how can you display a list of number at all during compile time?
// If they're represented as a single type and you throw during a constexpr. We can trust
// the compiler to emit the type.
// Well, std::integer_sequence should work.
//
// Ok, but how do you get from const int[] to an std::integer_sequence?
// You can convert to an std::array as an intermediary.
//
// So const int * -> std::array -> std::integer_sequence -> constexpr function that throws
// during compilation.
//
// Requires C++17

#include<array>
#include<tuple>

using namespace std;

template<int N>
constexpr auto ToArray(const int (&i)[N]){
  std::array<int, N> a = {{}};
  for (int j = 0; j < N; j++){
    a[j] = i[j];
  }
  return a;
}

template<typename A, typename B>
__attribute__((noinline))
__attribute__((error("\nwant, got")))
static constexpr void actually_got(void) {abort(); } // must

template <size_t N, typename F, size_t... indexes>
constexpr auto make_seq_helper(F f, std::index_sequence<indexes...> is) {
    return std::integer_sequence<int, std::get<indexes>(f())...>{};
}

template <typename F>
constexpr auto make_seq(F f) {
    constexpr size_t N = f().size();
    using indexes = std::make_index_sequence<N>;
    return make_seq_helper<N>(f, indexes{});
};

template <int SIZE, typename F, int N>
constexpr auto TestFill(const F f, const uint32_t (&i)[N]){

  std::array<int, SIZE> a = {{}};
  for (int j = 0; j < SIZE; j++){
    //constexpr auto k = f(i, j);
    a[j] = f(i,j);
  }
  return a;

};


template <typename F, typename G>
  constexpr int compareArrays(F y, G x){
  constexpr auto a = make_seq(y);
  constexpr auto b = make_seq(x);

  constexpr auto same =
    std::is_same<decltype(a),decltype(b)>::value;
  if constexpr (!same) {
     actually_got<decltype(a), decltype(b)>();
     return 0;
  }
 return 0;
};

// Example iterator to be tested
constexpr int DoubleSequence(const int* i, int n){
  return i[n] * 2;
}

/*
Example usage.
int main()
{
  constexpr const int x[] ={1,2,3,9,5};

  constexpr auto got =
    [](){return TestFill(DoubleSequence, x);};
  constexpr auto want =
    [](){return ToArray({2,4,6,8,10});};
  //  compile time compare arrays
  //  crash verbosely if not equal
  constexpr int test = compareArrays(got, want);
  }
*/
