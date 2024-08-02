// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.util.Precision;
import org.mockito.ArgumentMatcher;


public class AngleMatcher implements ArgumentMatcher<Double> {
    public enum Units {
      RADIANS, DEGREES;
    }

    private Units m_units;
    private double m_value;
    private double m_delta;

    public AngleMatcher(Units units, double value, double delta) {
      this.m_units = units;
      this.m_value = value;
      this.m_delta = delta;
    }

    @Override
    public boolean matches(Double number) {
      if (m_units.equals(Units.RADIANS) && Precision.equals(Math.abs(number), Math.PI, m_delta)) {
        return Precision.equals(Math.abs(number), Math.PI, m_delta);
      } else if (m_units.equals(Units.DEGREES) && Precision.equals(Math.abs(number), 180, m_delta)) {
        return Precision.equals(Math.abs(number), 180, m_delta);
      }

      return Precision.equals(number, m_value, m_delta)
          || Precision.equals(number + (m_units.equals(Units.RADIANS) ? Math.PI : 180), m_value, m_delta);
    }
}