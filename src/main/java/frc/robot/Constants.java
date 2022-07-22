// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public final static Translation2d FRONT_LEFT_LOCATION = new Translation2d();
        public final static Translation2d FRONT_RIGHT_LOCATION = new Translation2d();
        public final static Translation2d BACK_LEFT_LOCATION = new Translation2d();
        public final static Translation2d BACK_RIGHT_LOCATION = new Translation2d();

        public final static double WHEEL_CIRCUMFERENCE = 0;
        public final static double MAX_SPEED = 0;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
    }
}
