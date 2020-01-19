/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static double kTrackWidth = 0.91; // meters
        public static double kWheelRadius = 0.076; // meters
        public static double kShaftEncoderResolution = 2048; // counts per revolution bore encoder
        public static double positionChangePerRotation = 8.73; // Motor rotation per shaft rotation
        public static double kMaxVelocity = 13; // feet per second
        public static double kMaxAcceleration = 3; // Max Accel fet per second squared

        public static double kStatic = 0.131; // Constant feedforward term for the robot drive.
        public static double kV = 2.26; // Velocity-proportional feedforward term for the robot drive
        public static double kA = 0.384; //Acceleration-proportional feedforward term for the robot (.238)

        // Tuning parameter (b > 0) for which larger values make convergence more aggressive like a proportional term
        public static double kRamseteB = 2;

        // Tuning parameter (0 &lt; zeta &lt; 1) for which larger values provide more damping in response
        public static double kRamseteZeta = 0.7;

        public static double kHeadingP = 1;
        public static double kHeadingD = 0;
        public static double kHeadingI = 0;

        public static double kDriveP = 2.45; // 3 stable
        public static double kDriveI = 0;
        public static double kDriveD = 0;

        public static int[] kLeftEncoderPorts = new int[]{0, 1};
        public static int[] kRightEncoderPorts = new int[]{2, 3};

        public static boolean kLeftEncoderReversed = false;
        public static boolean kRightEncoderReversed = false;

        public static boolean kGyroReversed = true;
    }
}
