// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive extends SubsystemBase {
    // absolute encoder offsets (0-1 | 0-360)
    // backleft (ID 12): -0.625732421875 | -225.263671875
    // backright (ID 9): -0.5673828125 | -204.2578125
    // frontleft (ID 3): -0.422607421875 | -152.138671875
    // frontright (ID 6): -0.292236328125 | -105.205078125

    private final SwerveDrive swerve;

    private double maxSpeed = Units.feetToMeters(DriveConstants.MAX_SPEED);

    /** Creates a new Drive. */
    public Drive(File directory) {
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER),
                DriveConstants.DRIVE_GEAR_RATIO, DriveConstants.DRIVE_PULSE_PER_ROTATION);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(DriveConstants.ANGLE_GEAR_RATIO,
                DriveConstants.ANGLE_PULSE_PER_ROTATION);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerve = new SwerveParser(directory)
                    .createSwerveDrive(
                            Units.feetToMeters(DriveConstants.MAX_SPEED), driveConversionFactor, angleConversionFactor);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }

        swerve.setHeadingCorrection(false);

        setupPathPlanner();
    }

    /*
     * public Drive(SwerveDriveConfiguration driveConf,
     * SwerveControllerConfiguration controllerConf) {
     * swerve = new SwerveDrive(driveConf, controllerConf, maxSpeed);
     * }
     */

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                        // Constants class
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Translation PID constants
                        new PIDConstants(swerve.swerveController.config.headingPIDF.p,
                                swerve.swerveController.config.headingPIDF.i,
                                swerve.swerveController.config.headingPIDF.d),
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return AutoBuilder.followPath(path);
    }

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerve.getMaximumVelocity(), 4.0,
                swerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }

    /*
     * public Command driveCommand(DoubleSupplier translationX, DoubleSupplier
     * translationY, DoubleSupplier headingX,
     * DoubleSupplier headingY) {
     * // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
     * // correction for this kind of control.
     * return run(() -> {
     * double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll
     * out
     * double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll
     * out
     * // Make the robot move
     * driveFieldOriented(swerve.swerveController.getTargetSpeeds(xInput, yInput,
     * headingX.getAsDouble(),
     * headingY.getAsDouble(),
     * swerve.getOdometryHeading().getRadians(),
     * swerve.getMaximumVelocity()));
     * });
     * }
     */

    /*
     * public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier
     * translationY, DoubleSupplier rotation) {
     * // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
     * // correction for this kind of control.
     * return run(() -> {
     * // Make the robot move
     * driveFieldOriented(swerve.swerveController.getTargetSpeeds(translationX.
     * getAsDouble(),
     * translationY.getAsDouble(),
     * rotation.getAsDouble() * Math.PI,
     * swerve.getOdometryHeading().getRadians(),
     * swerve.getMaximumVelocity()));
     * });
     * }
     */

    /*
     * public Command driveCommand(DoubleSupplier translationX, DoubleSupplier
     * translationY,
     * DoubleSupplier angularRotationX) {
     * return run(() -> {
     * // Make the robot move
     * swerve.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) *
     * swerve.getMaximumVelocity(),
     * Math.pow(translationY.getAsDouble(), 3) * swerve.getMaximumVelocity()),
     * Math.pow(angularRotationX.getAsDouble(), 3) *
     * swerve.getMaximumAngularVelocity(),
     * true,
     * false);
     * });
     * }
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerve.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerve.driveFieldOriented(velocity);
    }

    public void set(ChassisSpeeds velocity) {
        swerve.drive(velocity);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerve.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerve.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerve.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        swerve.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        swerve.zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /*
     * public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double
     * headingX, double headingY) {
     * xInput = Math.pow(xInput, 3);
     * yInput = Math.pow(yInput, 3);
     * return swerve.swerveController.getTargetSpeeds(xInput,
     * yInput,
     * headingX,
     * headingY,
     * getHeading().getRadians(),
     * maxSpeed);
     * }
     */

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerve.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maxSpeed);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerve.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerve.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerve.swerveDriveConfiguration;
    }

    public void lock() {
        swerve.lockPose();
    }

    public Rotation2d getPitch() {
        return swerve.getPitch();
    }

    public void addFakeVisionReading() {
        swerve.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

}
