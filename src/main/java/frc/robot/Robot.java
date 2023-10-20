// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team6328.Alert;
import frc.lib.team6328.VirtualSubsystem;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.subsystems.leds.LEDs;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private Command m_subsystemCheckCommand;
    private RobotContainer m_robotContainer;

    private final Timer canErrorTimer = new Timer();
    private final Timer disabledTimer = new Timer();

    private final Alert logReceiverQueueAlert =
            new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
    private final Alert canErrorAlert =
            new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
    private final Alert lowBatteryAlert = new Alert(
            "Battery voltage is very low, consider turning off the robot or replacing the battery.",
            AlertType.WARNING);

    @Override
    public void robotInit() {
        Logger logger = Logger.getInstance();
        LEDs.getInstance();

        if (Constants.kIsReal) {
            String folder = "";
            logger.addDataReceiver(new WPILOGWriter(folder));
            logger.addDataReceiver(new NT4Publisher());
            LoggedPowerDistribution.getInstance(0, ModuleType.kRev);
        } else {
            logger.addDataReceiver(new NT4Publisher());
        }
        logger.start();


        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.getInstance().recordOutput(
                    "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
                    active);
            Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            logCommandFunction.accept(command, true);
        });
        CommandScheduler.getInstance().onCommandFinish((Command command) -> {
            logCommandFunction.accept(command, false);
        });
        CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
            logCommandFunction.accept(command, false);
        });


        // Start timers
        canErrorTimer.reset();
        canErrorTimer.start();
        disabledTimer.reset();
        disabledTimer.start();

        // Instantiate RobotContainer
        System.out.println("[Init] Instantiating RobotContainer");
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        VirtualSubsystem.periodicAll();

        // Check logging fault
        logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

        // Update CAN error alert
        var canStatus = RobotController.getCANStatus();
        if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
            canErrorTimer.reset();
        }
        canErrorAlert.set(!canErrorTimer.hasElapsed(0.5));

        // Update low battery alert
        if (DriverStation.isEnabled()) {
            disabledTimer.reset();
        }
        if (RobotController.getBatteryVoltage() < 10.0
                && disabledTimer.hasElapsed(2.0)) {
            LEDs.getInstance().lowBatteryAlert = true;
            lowBatteryAlert.set(true);
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_subsystemCheckCommand = m_robotContainer.getSubsystemCheckCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        } else if (!DriverStation.isFMSAttached() && m_subsystemCheckCommand != null) {
            System.out.println("SystemCheck Started");
            m_subsystemCheckCommand.schedule();
        } else {
            System.out.println("No Auto Rountine Selected!!!");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (m_subsystemCheckCommand != null) {
            m_subsystemCheckCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
