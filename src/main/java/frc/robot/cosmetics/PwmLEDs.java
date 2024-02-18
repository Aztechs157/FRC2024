// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cosmetics;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CosmeticConstants;

public class PwmLEDs extends SubsystemBase {
    private final AddressableLED lights = new AddressableLED(CosmeticConstants.LIGHT_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(CosmeticConstants.LIGHT_LENGTH);
    private Color color1 = Color.kBlack;
    private Color color2 = Color.kBlack;
    private double onLength = 0.0;
    private double offLength = 0.0;
    private int color1Length = 0;
    private int color2Length = 0;
    private double speed = 0;
    private double cycleLength = 0.0;
    private double duration = 0.0;

    public Color allianceColor = Color.kFirstRed;

    private double mp1 = 90.0;
    private double mp2 = 60.0;
    private double mp3 = 30.0;
    private double mp4 = 15.0;
    private double mp5 = 10.0;
    private double mp6 = 5.0;
    private double mpTolerance = 0.5;

    private Mode lightMode = Mode.SOLID;

    public void setLightMode(Mode lightMode) {
        this.lightMode = lightMode;
    }

    public Color getColor1() {
        return color1;
    }

    public void setColor1(Color color1) {
        this.color1 = color1;
    }

    public Color getColor2() {
        return color2;
    }

    public void setColor2(Color color2) {
        this.color2 = color2;
    }

    public void setColor1Length(int color1Length) {
        this.color1Length = color1Length;
    }

    public void setColor2Length(int color2Length) {
        this.color2Length = color2Length;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public static enum Mode {
        SOLID, WAVE, CLIMB, STROBE;
    }

    /** Creates a new PwmLEDs. */
    public PwmLEDs() {
        lights.setLength(CosmeticConstants.LIGHT_LENGTH);
        lights.setData(buffer);
        lights.start();
    }

    public void updateAllianceColor() {
        var alliance = DriverStation.getAlliance();
        allianceColor = alliance.get() == DriverStation.Alliance.Red ? Color.kFirstRed : Color.kFirstBlue;
    }

    public void solid(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    public void setSolid(Color color) {
        this.color1 = color;
        this.lightMode = Mode.SOLID;
    }

    public void wave(Color color1, Color color2, double cycleLength, double duration) {
        double counter = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double counterDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < buffer.getLength(); i++) {
            counter += counterDiffPerLed;
            if (i >= 0) {
                double ratio = (Math.pow(Math.sin(counter), 0.4) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(counter + Math.PI), 0.4) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) {
                    ratio = 0.5;
                }
                double red = (color1.red * (1 - ratio)) + (color2.red * ratio);
                double green = (color1.green * (1 - ratio)) + (color2.green * ratio);
                double blue = (color1.blue * (1 - ratio)) + (color2.blue * ratio);
                buffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    public void wave(Color color, double cycleLength, double duration) {
        wave(color, Color.kBlack, cycleLength, duration);
    }

    public void setWave(Color color1, Color color2, double cycleLength, double duration) {
        this.color1 = color1;
        this.color2 = color2;
        this.cycleLength = cycleLength;
        this.duration = duration;
        this.lightMode = Mode.WAVE;
    }

    public void climb(Color color1, Color color2, int color1Length, int color2Length, double speed) {
        int counter = (int) Math.floor(Timer.getFPGATimestamp() * speed);
        for (int i = 0; i < buffer.getLength(); i += color2Length + color1Length) {
            for (int j = 0; j < color1Length; j++) {
                buffer.setLED((i + j + counter) % buffer.getLength(), color1);
            }
            for (int j = color1Length; j < color1Length + color2Length; j++) {
                buffer.setLED((i + j + counter) % buffer.getLength(), color2);
            }
        }
    }

    public void climb(Color color, int colorLength, int offLength, double speed) {
        climb(color, Color.kBlack, colorLength, offLength, speed);
    }

    public void setClimb(Color color1, Color color2, int color1Length, int color2Length, double speed) {
        this.color1 = color1;
        this.color2 = color2;
        this.color1Length = color1Length;
        this.color2Length = color2Length;
        this.speed = speed;
        this.lightMode = Mode.CLIMB;
    }

    public void strobe(Color color1, Color color2, double onLength, double offLength) {
        boolean lightsOn = Timer.getFPGATimestamp() % onLength + offLength > onLength;
        if (lightsOn) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color1);
            }
        } else {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color2);
            }
        }
    }

    public void strobe(Color color, double onLength, double offLength) {
        strobe(color, Color.kBlack, onLength, offLength);
    }

    public void setStrobe(Color color1, Color color2, double onLength, double offLength) {
        this.color1 = color1;
        this.color2 = color2;
        this.onLength = onLength;
        this.offLength = offLength;
        this.lightMode = Mode.STROBE;
    }

    public void setDefault() {
        updateAllianceColor();
        Color color1 = Color.kBlue;
        Color color2 = Color.kGold;
        if (DriverStation.isEStopped()) {
            color1 = Color.kDarkGreen;
            color2 = Color.kPowderBlue;
        }
        if (!DriverStation.isFMSAttached()) {
            color1 = PwmLEDs.dimColor(color1, 0.25);
            color2 = PwmLEDs.dimColor(color2, 0.25);
        }

        setWave(color1, color2, 10, 3);
    }

    public static Color dimColor(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        double time = Timer.getMatchTime();
        if (Math.abs(time - mp1) <= mpTolerance) {
            solid(Color.kBlue);
            lights.setData(buffer);
            return;
        }
        if (Math.abs(time - mp2) <= mpTolerance) {
            solid(Color.kGreen);
            lights.setData(buffer);
            return;
        }
        if (Math.abs(time - mp3) <= mpTolerance) {
            solid(Color.kFirstRed);
            lights.setData(buffer);
            return;
        }
        if (time < mp4 && time > mp5 && time % 1.0 > 0.5) {
            solid(dimColor(Color.kDarkGoldenrod, 0.5));
            lights.setData(buffer);
            return;
        }
        if (time < mp5 && time > mp6 && time % 1.0 > 0.5) {
            solid(dimColor(Color.kFirstBlue, 0.5));
            lights.setData(buffer);
            return;
        }
        if (time < mp6 && time % 1.0 > 0.5) {
            solid(dimColor(Color.kFirstRed, 0.5));
            lights.setData(buffer);
            return;
        }

        switch (lightMode) {
            case SOLID:
                solid(color1);
                break;
            case WAVE:
                wave(color1, color2, cycleLength, duration);
                break;
            case CLIMB:
                climb(color1, color2, color1Length, color2Length, speed);
                break;
            case STROBE:
                strobe(color1, color2, onLength, offLength);
                break;
            default:
                solid(Color.kBlack);
        }

        lights.setData(buffer);
    }
}
