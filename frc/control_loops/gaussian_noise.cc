#include "frc/control_loops/gaussian_noise.h"

namespace frc::control_loops {

GaussianNoise::GaussianNoise(unsigned int seed, double stddev)
    : stddev_(stddev), generator_(seed), distribution_(0.0, 1.0) {
  // Everything is initialized now.
}

double GaussianNoise::AddNoiseToSample(double sample) {
  return sample + (distribution_(generator_) * stddev_);
}

}  // namespace frc::control_loops
