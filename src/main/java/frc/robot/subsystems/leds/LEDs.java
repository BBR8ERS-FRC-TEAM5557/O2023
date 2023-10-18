// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.ObjectiveTracker;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import java.util.List;


public class LEDs extends VirtualSubsystem {

    private static LEDs instance;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    // Robot state tracking
    public int loopCycleCount = 0;
    public HPGamePiece hpGamePiece = HPGamePiece.NONE;
    public boolean gripperStopped = false;
    public boolean endgameAlert = false;
    public boolean lowBatteryAlert = false;
    private Alliance alliance = Alliance.Invalid;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final Notifier loadingNotifier;

    // Constants
    private static final int minLoopCycleCount = 10;
    private static final int length = 36;
    private static final double strobeSlowDuration = 0.2;
    private static final double breathDuration = 1.0;
    private static final double waveExponent = 0.4;
    private static final double waveFastCycleLength = 25.0;
    private static final double waveFastDuration = 0.25;
    private static final double waveSlowDuration = 3.0;
    private static final double waveAllianceCycleLength = 15.0;
    private static final double waveAllianceDuration = 2.0;

    private LEDs() {
        System.out.println("[Init] Creating LEDs");
        leds = new AddressableLED(Constants.RobotMap.kLedsDIO);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
        loadingNotifier = new Notifier(() -> {
            synchronized (this) {
                breath(Section.FULL, Color.kWhite, Color.kBlack, 0.25,
                        System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
            }
        });
        loadingNotifier.startPeriodic(0.02);
    }

    public synchronized void periodic() {
        // Update alliance color
        if (DriverStation.isFMSAttached()) {
            alliance = DriverStation.getAlliance();
        }

        //Update current gamepiece
        if(ObjectiveTracker.getGamePiece() == GamePiece.CONE) {
            hpGamePiece = HPGamePiece.CONE;
        } else {
            hpGamePiece = HPGamePiece.CUBE;
        }

        // Exit during initial cycles
        loopCycleCount += 1;
        if (loopCycleCount < minLoopCycleCount) {
            return;
        }

        // Stop loading notifier if running
        loadingNotifier.stop();

        // Default to off
        solid(Section.FULL, Color.kAliceBlue);

        if (DriverStation.isDisabled()) {
            if (lowBatteryAlert) {
                // Low battery
                solid(Section.FULL, Color.kOrangeRed);
            } else {
                // Default pattern
                switch (alliance) {
                    case Red:
                        wave(Section.FULL, Color.kRed, Color.kBlack, waveAllianceCycleLength,
                                waveAllianceDuration);
                        break;
                    case Blue:
                        wave(Section.FULL, Color.kBlue, Color.kBlack, waveAllianceCycleLength,
                                waveAllianceDuration);
                        break;
                    default:
                        stripes(Section.FULL, List.of(Color.kDarkBlue, Color.kWhite, Color.kRed), 4,
                                waveSlowDuration);
                        break;
                }
            }
        } else if (DriverStation.isAutonomous()) {
            wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
        } else {
            // Set HP indicator
            Color hpColor = null;
            switch (hpGamePiece) {
                case CUBE:
                    hpColor = Color.kPurple;
                    break;
                case CONE:
                    hpColor = Color.kGold;
                    break;
                case NONE:
                    break;
                default:
                    break;
            }
            breath(Section.FULL, hpColor, Color.kBlack, 1.0);
        }

        // Set special modes
        if (endgameAlert) {
            strobe(Section.FULL, Color.kBlue, strobeSlowDuration);
        } else if (gripperStopped) {
            solid(Section.FULL, Color.kGreen);
        }

        // Update LEDs
        leds.setData(buffer);
    }

    private void solid(Section section, Color color) {
        if (color != null) {
            for (int i = section.start(); i < section.end(); i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private void strobe(Section section, Color color, double duration) {
        boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(section, on ? color : Color.kBlack);
    }

    private void breath(Section section, Color c1, Color c2, double duration) {
        breath(section, c1, c2, duration, Timer.getFPGATimestamp());
    }

    private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
        double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        solid(section, new Color(red, green, blue));
    }

    private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < section.end(); i++) {
            x += xDiffPerLed;
            if (i >= section.start()) {
                double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) {
                    ratio = 0.5;
                }
                double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
                double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
                double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
                buffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    private void stripes(Section section, List<Color> colors, int length, double duration) {
        int offset =
                (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
        for (int i = section.start(); i < section.end(); i++) {
            int colorIndex = (int) (Math.floor((double) (i - offset) / length) + colors.size())
                    % colors.size();
            colorIndex = colors.size() - 1 - colorIndex;
            buffer.setLED(i, colors.get(colorIndex));
        }
    }

    public static enum HPGamePiece {
        NONE, CUBE, CONE
    }

    private static enum Section {
        FULL;

        private int start() {
            switch (this) {
                case FULL:
                    return 0;
                default:
                    return 0;
            }
        }

        private int end() {
            switch (this) {
                case FULL:
                    return length;
                default:
                    return length;
            }
        }
    }
}
