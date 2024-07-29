// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.util.Precision;
import org.mockito.ArgumentMatcher;


public class AbsoluteValueMatcher implements ArgumentMatcher<Double> {
    private double m_value;
    private double m_delta;

    public AbsoluteValueMatcher(double value, double delta) {
      this.m_value = value;
      this.m_delta = delta;
    }

    @Override
    public boolean matches(Double number) {
      return Precision.equals(+number, m_value, m_delta) || Precision.equals(-number, m_value, m_delta);
    }
}