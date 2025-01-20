#ifndef FRC_WPILIB_ADIS16448_H_
#define FRC_WPILIB_ADIS16448_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "frc/wpilib/ahal/DigitalInput.h"
#include "frc/wpilib/ahal/DigitalOutput.h"
#include "frc/wpilib/ahal/SPI.h"
#undef ERROR

#include "aos/events/shm_event_loop.h"
#include "aos/logging/logging.h"
#include "frc/wpilib/fpga_time_conversion.h"
#include "frc/wpilib/imu_batch_generated.h"
#include "frc/wpilib/imu_generated.h"
#include "frc/wpilib/spi_rx_clearer.h"

namespace frc::wpilib {

// Handles interfacing with an Analog Devices ADIS16448 Inertial Sensor over
// SPI and sending values out on a queue.
//
// The sensor is configured to generate samples at 204.8 Hz, and the values are
// sent out as each sample is received.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class ADIS16448 {
 public:
  // port is where to find the sensor over SPI.
  // dio1 must be connected to DIO1 on the sensor.
  ADIS16448(::aos::ShmEventLoop *event_loop, frc::SPI::Port port,
            frc::DigitalInput *dio1);

  // Sets the dummy SPI port to send values on to make the roboRIO deassert the
  // chip select line. This is mainly useful when there are no other devices
  // sharing the bus.
  void SetDummySPI(frc::SPI::Port port);

  // Sets the reset line for the IMU to use for error recovery.
  void set_reset(frc::DigitalOutput *output) { reset_ = output; }

  // Sets a function to be called immediately after each time this class uses
  // the SPI bus. This is a good place to do other things on the bus.
  void set_spi_idle_callback(std::function<void()> spi_idle_callback) {
    spi_idle_callback_ = std::move(spi_idle_callback);
  }

 private:
  // Initializes the sensor and then takes readings until Quit() is called.
  void DoRun();

  // Try to initialize repeatedly as long as we're supposed to be running.
  void InitializeUntilSuccessful();

  // Converts a 16-bit value at data to a scaled output value where a value of 1
  // corresponds to lsb_per_output.
  float ConvertValue(uint8_t *data, double lsb_per_output, bool sign = true);

  // Performs an SPI transaction.
  // Returns true if it succeeds.
  template <uint8_t size>
  bool DoTransaction(uint8_t to_send[size], uint8_t to_receive[size]);

  // Reads one of the gyro's registers and returns the value in value.
  // next_address is the address of the *next* register to read.
  // Not sure what gets stored in value for the first read, but it should be
  // ignored. Passing nullptr for value is allowed to completely ignore it.
  // Returns true if it succeeds.
  bool ReadRegister(uint8_t next_address, uint16_t *value);

  // Writes a value to one of the registers.
  // Returns true if it succeeds.
  bool WriteRegister(uint8_t address, uint16_t value);

  // Checks the given value of the DIAG_STAT register and logs any errors.
  // Returns true if there are no errors we care about.
  bool CheckDiagStatValue(uint16_t value) const;

  // Starts everything up and runs a self test.
  // Returns true if it succeeds.
  bool Initialize();

  ::aos::EventLoop *event_loop_;
  ::aos::Sender<::frc::IMUValuesBatch> imu_values_sender_;

  // TODO(Brian): This object has no business owning these ones.
  const ::std::unique_ptr<frc::SPI> spi_;
  ::std::unique_ptr<frc::SPI> dummy_spi_;
  frc::DigitalInput *const dio1_;
  frc::DigitalOutput *reset_ = nullptr;

  std::function<void()> spi_idle_callback_ = []() {};

  SpiRxClearer rx_clearer_;

  FpgaTimeConverter time_converter_;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_ADIS16448_H_
