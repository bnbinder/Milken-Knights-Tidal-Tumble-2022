// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

/** Add your docs here. */
public class MkCANCoder {
    private CANCoder encoder;
    public MkCANCoder(int port, double offset, boolean inverted, AbsoluteSensorRange range)
    {
        encoder = new CANCoder(port);
        encoder.configAbsoluteSensorRange(range);
        encoder.configSensorDirection(inverted);
        encoder.configMagnetOffset(offset);
    }

    public double getAbsPosition()
    {
        return encoder.getAbsolutePosition();
    }

    public void configEncoderOffset(double offset)
    {
        encoder.configMagnetOffset(offset);
    }
}
