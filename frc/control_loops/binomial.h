#ifndef FRC_CONTROL_LOOPS_BINOMIAL_H_
#define FRC_CONTROL_LOOPS_BINOMIAL_H_

namespace frc::control_loops {

// Computes the factorial of n
constexpr double Factorial(int n) {
  if (n <= 1) {
    return 1.0;
  } else {
    return Factorial(n - 1) * n;
  }
}

// Computes the binomial coefficients.  n choose k.
constexpr double Binomial(int n, int k) {
  return Factorial(n) / (Factorial(k) * Factorial(n - k));
}

}  // namespace frc::control_loops

#endif  // FRC_CONTROL_LOOPS_BINOMIAL_H_
