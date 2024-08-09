// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;

import org.assabet.aztechs157.numbers.Range;
import org.assabet.aztechs157.numbers.RangeConverter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class DeflectorSystem extends SubsystemBase {

    private CANSparkFlex deflectorMotor = new CANSparkFlex(ShooterConstants.DEFLECTOR_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput deflectorExtLim = new DigitalInput(ShooterConstants.DEFLECTOR_EXT_LIM_ID);
    private DigitalInput deflectorRetLim = new DigitalInput(ShooterConstants.DEFLECTOR_RET_LIM_ID);
    private AnalogInput deflectorPot = new AnalogInput(ShooterConstants.DEFLECTOR_POT_ID);

    private Range deflectorActualRange = new Range(3650, 1900);
    private Range zeroToHundred = new Range(0, 100);
    private RangeConverter deflectorRange = new RangeConverter(deflectorActualRange, zeroToHundred);

    /** Creates a new DeflectorSystem. */
    public DeflectorSystem(boolean debugModeOn) {
        deflectorMotor.setIdleMode(IdleMode.kBrake);

        if (debugModeOn) {
            Shuffleboard.getTab("Deflector").addBoolean("readExtLimitSwitch", this::readExtLimitSwitch);
            Shuffleboard.getTab("Deflector").addBoolean("readRetLimitSwitch", this::readRetLimitSwitch);
            Shuffleboard.getTab("Deflector").addInteger("readPot", this::readDeflectorPot);
            Shuffleboard.getTab("Deflector").addDouble("readPotMapped", this::readDeflectorPotMappedInt);
        }
    }

    public void set(double velocity) {
        deflectorMotor.set(velocity);
    }

    public boolean readExtLimitSwitch() {
        return !deflectorExtLim.get();
    }

    public boolean readRetLimitSwitch() {
        return !deflectorRetLim.get();
    }

    public int readDeflectorPot() {
        return deflectorPot.getValue();
    }

    public double readDeflectorPotMapped() {
        return deflectorRange.convert(readDeflectorPot());
    }

    public int readDeflectorPotMappedInt() {
        return (int) Math.floor(deflectorRange.convert(readDeflectorPot()));
    }

    public double getDeflectorMotorVelocity() {
        return deflectorMotor.getEncoder().getVelocity();
    }

    public double deflectorMotorPID(double desiredPos) {
        double currentPos = readDeflectorPotMapped();
        return ShooterConstants.DEFLECTOR_MOTOR_PID.calculate(-currentPos, desiredPos);
    }

}
