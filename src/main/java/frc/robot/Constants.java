package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {

    public static class DriveConstants {
        public static final boolean FIELD_ORIENTED_ENABLED = true;

        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

        public record XboxSpeeds(
                double drive,
                double slowDrive,
                double rotate) {
            public static final XboxSpeeds COMPETITION = new XboxSpeeds(1, 0.5, 130);
            public static final XboxSpeeds DEMO = new XboxSpeeds(0.3, 0.3, 70);
        }

    }
}
