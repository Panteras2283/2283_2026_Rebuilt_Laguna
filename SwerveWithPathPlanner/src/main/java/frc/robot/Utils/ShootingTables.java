// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShootingTables {

    public static InterpolatingDoubleTreeMap FlywheelMap = new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap ExitVelocityMap = new InterpolatingDoubleTreeMap();

    static{
        /*Close Range shot */
        FlywheelMap.put(2.0, 3000.0);
        ExitVelocityMap.put(2.0, 12.0);

        /*Mid Range shot */
        FlywheelMap.put(3.5, 4500.0);
        ExitVelocityMap.put(3.5, 20.0);

        /*Long Range shot */
        FlywheelMap.put(6.0, 6000.0);
        ExitVelocityMap.put(6.0, 26.0);
    }
}
