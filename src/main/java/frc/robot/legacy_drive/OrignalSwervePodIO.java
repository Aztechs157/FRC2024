package frc.robot.legacy_drive;

import org.assabet.aztechs157.SwervePod.DebugKey;
import org.assabet.aztechs157.SwervePod.SwervePodIO;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants.LegacyDriveConstants;

public class OrignalSwervePodIO implements SwervePodIO {

    public record Config(int driveMotorId, int angleMotorId, int angleEncoderId, boolean driveMotorInverted) {
    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    private final CANcoder angleEncoder;
    private final NetworkTable table;

    public OrignalSwervePodIO(final Config config, final NetworkTable table) {
        this.table = table;

        driveMotor = new CANSparkMax(config.driveMotorId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(config.angleMotorId, MotorType.kBrushless);
        angleEncoder = new CANcoder(config.angleEncoderId);

        table.getEntry("Inverted").setBoolean(config.driveMotorInverted);

        driveMotor.setIdleMode(LegacyDriveConstants.DRIVE_IDLE_MODE);
        driveMotor.setInverted(false);
        angleMotor.setIdleMode(LegacyDriveConstants.ANGLE_IDLE_MODE);
        angleMotor.setInverted(true);
    }

    private final SlewRateLimiter driveSlewRate = new SlewRateLimiter(LegacyDriveConstants.DRIVE_SLEW_RATE);

    @Override
    public void setDriveSpeed(double speed) {
        if (table.getEntry("DriveInverted").getBoolean(false)) {
            speed = -speed;
        }

        driveMotor.set(driveSlewRate.calculate(speed));
    }

    @Override
    public void setAngleSpeed(final double speed) {
        angleMotor.set(speed);
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    @Override
    public Rotation2d getAnglePosition() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private final PIDController anglePid = new PIDController(
            LegacyDriveConstants.ANGLE_KP, 0, LegacyDriveConstants.ANGLE_KD);

    @Override
    public double calculateAnglePid(double measurement, double setpoint) {
        return anglePid.calculate(measurement, setpoint);
    }

    @Override
    public void debugValue(DebugKey key, double value) {
        table.getEntry(key.name()).setDouble(value);
    }
}
