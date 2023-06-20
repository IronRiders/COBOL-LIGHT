package team.ironriders.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Random;

public class LightsSubsystem extends SubsystemBase {

    public enum LightPattern {
        CONE,
        CUBE,
        GREEN,
        YELLOW,
        RAINBOW,
        NOISE,
        CUSTOM
    }

    private static LightPattern lightPattern =
            LightPattern.RAINBOW;

    AddressableLED addressableLed = new AddressableLED(0);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(50);
    SendableChooser<String> lightPatternChooser = new SendableChooser<>();
    SendableChooser<String> rainbowSpeedChooser = new SendableChooser<>();
    String lastSelectedLightPattern;
    int rainbowFirstLedHue = 0;
    int rainbowSpeed = 1;
    int customR = 255;
    int customG = 255;
    int customB = 255;

    ChassisSpeeds cs = new ChassisSpeeds();

    public LightsSubsystem() {
        addressableLed.setLength(ledBuffer.getLength());

        lightPatternChooser.setDefaultOption("Rainbow", "RAINBOW");
        lightPatternChooser.addOption("Cone", "CONE");
        lightPatternChooser.addOption("Cube", "CUBE");
        lightPatternChooser.addOption("Green", "GREEN");
        lightPatternChooser.addOption("Yellow", "YELLOW");
        lightPatternChooser.addOption("Noise", "NOISE");
        lightPatternChooser.addOption("Custom Color", "CUSTOM");

        SmartDashboard.putData("Light Patterns", lightPatternChooser);

        rainbowSpeedChooser.setDefaultOption("Speed 1", "1");
        for (int i = 2; i < 15; i++) {
            rainbowSpeedChooser.addOption(String.format("Speed %d", i), String.valueOf(i));
        }

        SmartDashboard.putData("Rainbow Speed", rainbowSpeedChooser);

        SmartDashboard.putNumber("Custom R", 255);
        SmartDashboard.putNumber("Custom G", 255);
        SmartDashboard.putNumber("Custom B", 255);
        SmartDashboard.putBoolean("Velocity Based Brightness", false);
    }

    @Override
    public void periodic() {
        // Runs only when value is changed so that other things can make it change freely
        if (!lightPatternChooser.getSelected().equals(lastSelectedLightPattern)) {
            setLightPattern(LightPattern.valueOf(lightPatternChooser.getSelected()));
            lastSelectedLightPattern = lightPatternChooser.getSelected();
        }

        rainbowSpeed = Integer.parseInt(rainbowSpeedChooser.getSelected());

        customR = (int) SmartDashboard.getNumber("Custom R", 255);
        customG = (int) SmartDashboard.getNumber("Custom G", 255);
        customB = (int) SmartDashboard.getNumber("Custom B", 255);

        switch (lightPattern) {
            case CONE:
            case YELLOW:
                setColorRGB(255, 255, 0);
                break;
            case CUBE:
                setColorRGB(255, 0, 255);
                break;
            case GREEN:
                setColorRGB(0, 200, 0);
                break;
            case RAINBOW:
                rainbow();
                break;
            case NOISE:
                noise();
                break;
            case CUSTOM:
                setColorRGB(customR, customG, customB);
        }
        addressableLed.start();
    }

    private void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = (rainbowFirstLedHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstLedHue += rainbowSpeed;
        rainbowFirstLedHue %= 180;
        addressableLed.setData(ledBuffer);
    }

    private void noise() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(
                    i, new Random().nextInt(256), new Random().nextInt(256), new Random().nextInt(256));
        }
        addressableLed.setData(ledBuffer);
    }

    public void setColorRGB(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, b, g); // BUG ON WPILIB: MIXES BLUE AND GREEN
        }
        addressableLed.setData(ledBuffer);
    }

    public void setColorHSV(int h, int s, int v) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, h, s, v);
        }
        addressableLed.setData(ledBuffer);
    }

    public void setLightPattern(LightPattern newLightPattern) {
        lightPattern = newLightPattern;
    }

    public void setChassisSpeeds(ChassisSpeeds cs) {
        this.cs = cs;
    }
}
