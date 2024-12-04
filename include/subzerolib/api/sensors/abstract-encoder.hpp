#pragma once

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

struct AbstractEncoder {
  /// @brief get the degree measurement
  /// @returns the degrees rotated
  virtual double get_deg() = 0;

  /// @brief set a new measurement
  /// @param deg the position in degrees
  virtual void set_deg(double deg) = 0;
};

struct AbstractRotationEncoder : public AbstractEncoder, public pros::Rotation {
  /// @brief create an object to forward function calls to a rotation sensor
  /// @param iport the port of the sensor
  /// @returns an abstract encoder object
  AbstractRotationEncoder(std::uint8_t iport) : pros::Rotation(iport) {
    pros::Rotation::set_data_rate(5);
  }

  /// @brief get the degree measurement
  /// @returns the degrees rotated
  double get_deg() override { return pros::Rotation::get_position() / 100.0; }

  /// @brief set a new measurement
  /// @param deg the position in degrees
  void set_deg(double deg) override { pros::Rotation::set_position(100 * deg); }
};

// TODO:
struct AbstractPotentiometerEncoder : public AbstractEncoder,
                                      public pros::adi::Potentiometer {
  AbstractPotentiometerEncoder(std::uint8_t adi_port)
      : pros::adi::Potentiometer(adi_port) {}
};

// forward function calls
class AbstractADIEncoder : public AbstractEncoder, public pros::adi::Encoder {
public:
  /// @brief create an object to forward function calls to an optical shaft
  /// encoder
  /// @param adi_port_top the top port
  /// @param adi_port_top the bottom port
  /// @param reversed if it should be reversed
  /// @returns an abstract encoder object
  AbstractADIEncoder(std::uint8_t adi_port_top,
                     std::uint8_t adi_port_bottom,
                     bool reversed = false)
      : pros::adi::Encoder(adi_port_top, adi_port_bottom, reversed) {}

  /// @brief create an object to forward function calls to an optical shaft
  /// encoder connected via expander
  /// @param tuple a pros::adi::ext_adi_port_tuple_t type struct specifying the
  /// tuple
  /// @param reversed if it should be reversed
  /// @returns an abstract encoder object
  AbstractADIEncoder(pros::adi::ext_adi_port_tuple_t tuple,
                     bool reversed = false)
      : pros::adi::Encoder(tuple, reversed) {}

  /// @brief get the degree measurement
  /// @returns the degrees rotated
  double get_deg() override { return pros::adi::Encoder::get_value() + offset; }

  /// @brief set a new measurement
  /// @param deg the position in degrees
  void set_deg(double deg) override {
    pros::adi::Encoder::reset();
    offset = deg;
  }

private:
  double offset = 0.0;
};