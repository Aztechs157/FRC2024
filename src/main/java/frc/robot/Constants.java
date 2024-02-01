package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {

    public static class ControllerConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6; // TODO: figure out if this is a good value

    }

    public static class DriveConstants {

        public static final boolean FIELD_ORIENTED_ENABLED = true;

        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        // TODO: find proper values for the following three
        public static final double ROBOT_MASS = (80) * 0.453592; // lbs * kg per pound
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag

        public static final double WHEEL_DIAMETER = 3.95;
        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double DRIVE_PULSE_PER_ROTATION = 42;
        public static final double ANGLE_GEAR_RATIO = 12.8; // This is the Gear Ratio of the motor encoder. Not
                                                            // absolute.
        public static final double ANGLE_PULSE_PER_ROTATION = 42; // This is the pulse/rev of the motor encoder. Not
                                                                  // absolute.
        public static final double MAX_SPEED = 12.2;

        public record XboxSpeeds(
                double drive,
                double slowDrive) {
            public static final XboxSpeeds COMPETITION = new XboxSpeeds(1, 0.5);
            public static final XboxSpeeds DEMO = new XboxSpeeds(0.3, 0.3);
        }

    }

    public static class VisionConstants {

        public static final String CAMERA_NICKNAME = ""; // TODO: find proper value
        public static final Transform3d CAMERA_PLACEMENT = new Transform3d(); // TODO: find proper value

    }

    public static class ShooterConstants {

        public static final int SHOOTER_MOTOR_LEFT_ID = 13;
        public static final int SHOOTER_MOTOR_RIGHT_ID = 14;

    }

    public static class PneumaticsConstants {

        public static final int COMPRESSOR_ID = 21;

        public static final int SOLENOID_FORWARD_CHANNEL = 0;
        public static final int SOLENOID_REVERSE_CHANNEL = 1;

    }

}
