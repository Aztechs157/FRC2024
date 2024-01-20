package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {

    public static class DriveConstants {
        public static final boolean FIELD_ORIENTED_ENABLED = true;

        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

        public static final double WHEEL_DIAMETER = 3.5;
        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double DRIVE_PULSE_PER_ROTATION = 42;
        public static final double ANGLE_GEAR_RATIO = 1;
        public static final double ANGLE_PULSE_PER_ROTATION = 1;
        public static final double MAX_SPEED = 17.3;

        public record XboxSpeeds(
                double drive,
                double slowDrive) {
            public static final XboxSpeeds COMPETITION = new XboxSpeeds(1, 0.5);
            public static final XboxSpeeds DEMO = new XboxSpeeds(0.3, 0.3);
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
}
