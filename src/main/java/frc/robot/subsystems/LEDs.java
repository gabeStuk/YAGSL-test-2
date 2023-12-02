// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OneMechanism;
import frc.robot.commands.LEDs.LarsonAnimationV2;
import frc.robot.commands.LEDs.SlowAlternateBlink;
import frc.robot.commands.LEDs.LarsonAnimationV2.V2BounceMode;

public class LEDs extends SubsystemBase {

    public enum CANdleMode {
        VICTORY_SPIN, //
        FIRE, //
        ACTIVE, //
        IDLE, //
        SLIDE
    }

    public enum Color {
        GREEN(0, 254, 0), //
        PURPLE(118, 0, 254), //
        ORANGE(254, 55, 0), //
        BLUE(0, 0, 254), //
        WHITE(254, 254, 254), //
        RED(254, 0, 0), //
        OFF(0, 0, 0);

        private int r;
        private int g;
        private int b;

        private Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private CANdleMode currentMode;
    private Color color, lastColor;
    private Color beaconColor = Color.RED;

    private final List<Supplier<Animation>> currentAnimations;

    private final int NUM_LEDS = 119;
    private final int STRIP_LENGTH = 51;

    private CANdle candle;
    private boolean beacon = false;
    private boolean fade = false;
    private boolean locked = false;

    private static LEDs instance;


    /** Creates a new LEDs. */
    public LEDs() {
        currentMode = CANdleMode.ACTIVE;
        currentAnimations = new ArrayList<>();
        for (int i = 0; i < 4; i++)
            currentAnimations.add(() -> null);
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.5);
        candle.configLEDType(LEDStripType.GRB);
        candle.configLOSBehavior(true);

        setColor(Color.ORANGE);
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public void setBlank() {
        setColor(Color.OFF);
    }

    public SlowAlternateBlink setClimb() {
        return new SlowAlternateBlink(color, Color.GREEN, .5, instance);
    }

    public static LEDs getInstance() {
        return instance == null ? instance = new LEDs() : instance;
    }

    public SequentialCommandGroup blink() {
        return blink(color);
    }

    public SequentialCommandGroup blink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)));
    }

    public SequentialCommandGroup alternateBlink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(lastColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(lastColor)),
            new WaitCommand(0.02));
    }

    public SequentialCommandGroup blinkWhite() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                switch (OneMechanism.getGamePieceMode()) {
                    case PURPLE_CUBE:
                        lastColor = Color.PURPLE;
                        break;
                    case ORANGE_CONE:
                        lastColor = Color.ORANGE;
                        break;
                    default:
                        lastColor = Color.ORANGE;
                        break;
                }
            }),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(lastColor)));
    }

    public SequentialCommandGroup blinkBeaconWhite() {
        Color lastBeaconColor = OneMechanism.getBeaconState() ? beaconColor : Color.OFF;
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(lastBeaconColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(lastBeaconColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> {
                setBeaconColor(lastBeaconColor);
                setBeaconState(lastBeaconColor != Color.OFF);
            }));
    }

    public SequentialCommandGroup blinkMulti(int iterations, Color... colors) {
        SequentialCommandGroup cmd = new SequentialCommandGroup();
        for (int i = 0; i < iterations; i++) {
            for (int j = 0; j < colors.length; j++) {
                Color tempColor = colors[j];
                cmd.addCommands(
                    runOnce(() -> setBlank()),
                    new WaitCommand(0.08),
                    runOnce(() -> setColor(tempColor)),
                    new WaitCommand(0.08));
            }
        }
        return cmd;
    }

    public void setBeaconColor(Color color) {
        setBeaconState(true);
        beaconColor = color;
        candle.setLEDs(color.r, color.g, color.b, 0, 8, 8);
        candle.setLEDs(color.r, color.g, color.b, 0, NUM_LEDS - 8, 8);
    }

    public void setBeaconState(boolean state) {
        beacon = state;
    }

    public void setFade(boolean fade) {
        clearAnimations();
        locked = false;
        this.fade = fade;
        if (fade) {
            currentAnimations.set(0, () -> new SingleFadeAnimation(color.r, color.g, color.b, 0, 0.8,
                    OneMechanism.getBeaconState() ? STRIP_LENGTH : NUM_LEDS, OneMechanism.getBeaconState() ? 16 : 0));

            currentAnimations.set(1,
                () -> OneMechanism.getBeaconState() ? new SingleFadeAnimation(color.r, color.g, color.b, 0, 0.8,
                STRIP_LENGTH + 1, STRIP_LENGTH + 8) : null);
        } else {
            if(currentMode != CANdleMode.ACTIVE) {
                switch (currentMode) {
                    case SLIDE: 
                        setSlide();
                        break;
                    case VICTORY_SPIN:
                        setVictorySpin();
                        break;
                    case FIRE:
                        setFireWorkPlz();
                        break;
                    default:
                        setIdle();
                        break;
                }
            }
        }
    }

    public void setLocked(boolean locked) {
        fade = false;
        clearAnimations();
        this.locked = locked;
        if (locked)
            currentAnimations.set(0, () -> new StrobeAnimation(color.r, color.g, color.b, 0, 0.2, NUM_LEDS));
        else {
            if(currentMode != CANdleMode.ACTIVE) {
                switch (currentMode) {
                    case SLIDE: 
                        setSlide();
                        break;
                    case VICTORY_SPIN:
                        setVictorySpin();
                        break;
                    case FIRE:
                        setFireWorkPlz();
                        break;
                    default:
                        setIdle();
                        break;
                }
            }
        }
    }

    public boolean getFade() {
        return fade;
    }

    public Command setIdle() {
        locked = false;
        fade = false;
        currentMode = CANdleMode.IDLE;
        clearAnimations();
        return new ParallelCommandGroup(
            // Left
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, false, 59, 8, 8, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, false, 59, 8, 16, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, false, 59, 8, 24, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, false, 59, 8, 32, 8,
                V2BounceMode.FRONT, this),
            // Right
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 8, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 16, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 24, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 32, 8,
                V2BounceMode.FRONT, this));
    }

    public void setSlide() {
        locked = false;
        fade = false;
        currentMode = CANdleMode.SLIDE;
        clearAnimations();

        currentAnimations.set(0, () -> new ColorFlowAnimation(color.r, color.g, color.b, 0, 0.88, NUM_LEDS,
            Direction.Forward, OneMechanism.getBeaconState() ? 16 : 8));

        currentAnimations.set(1,
            () -> new ColorFlowAnimation(color.r, color.g, color.b, 0, 0.88, NUM_LEDS, Direction.Backward,
                OneMechanism.getBeaconState() ? 8 : 0));
    }

    public void clearAnimations() {
        for (int i = 0; i < currentAnimations.size(); i++)
            currentAnimations.set(i, () -> null);
    }

    public void setVictorySpin() {
        locked = false;
        fade = false;
        currentMode = CANdleMode.VICTORY_SPIN;
        clearAnimations();;

        currentAnimations.set(0, () -> new RainbowAnimation(1, 1, NUM_LEDS));
    }

    public void setFireWorkPlz() {
        locked = false;
        fade = false;
        currentMode = CANdleMode.FIRE;
        clearAnimations();

        currentAnimations.set(0, () -> new FireAnimation(1.0, 0.2, STRIP_LENGTH, 0.4,
            0.3, true, 8));
        currentAnimations.set(1,
            () -> new FireAnimation(1.0, 0.2, STRIP_LENGTH, 0.4, 0.3, false, NUM_LEDS - STRIP_LENGTH));
    }

    public void setActive() {
        locked = false;
        fade = false;
        currentMode = CANdleMode.ACTIVE;
        clearAnimations();
    }

    public boolean getBeaconState() {
        return beacon;
    }

    public CANdleMode getMode() {
        return currentMode;
    }

    public Color getColor() {
        return color;
    }

    public CANdle getCandle() {
        return candle;
    }

    public int getNumLEDs() {
        return NUM_LEDS;
    }

    @Override
    public void periodic() {
        if (currentMode == CANdleMode.ACTIVE) {
            if (!OneMechanism.getScoreMode() && !beacon && !fade && !locked) {
                candle.setLEDs(color.r, color.g, color.b);
            } else if (!OneMechanism.getScoreMode() && !fade && !locked) {
                candle.setLEDs(color.r, color.g, color.b, 0, 0, 8);
                candle.setLEDs(color.r, color.g, color.b, 0, 16, STRIP_LENGTH - 8);
                candle.setLEDs(color.r, color.g, color.b, 0, NUM_LEDS - STRIP_LENGTH, STRIP_LENGTH - 8);
            }
        } else if (currentMode == CANdleMode.FIRE) {
            if (!OneMechanism.getScoreMode() && !beacon && !fade && !locked) {
                candle.setLEDs(color.r, color.g, color.b, 0, 0, 8);
            }
        }

        OneMechanism.checkAuxiliaryModesPeriodic();

        for (int i = 0; i < currentAnimations.size(); i++) {
            candle.animate(currentAnimations.get(i).get(), i);
        }
    }
}
