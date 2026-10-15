// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ScoringSetpoints;
import frc.robot.Constants.AutoAlign.CLIMB_MOVEMENT_SP;
import frc.robot.Constants.Climb.CLIMB_SETPOINT;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.utils.ClimbState;
import frc.robot.utils.StateMachine;
import poplib.controllers.io.XboxIO;
import poplib.math.MathUtil;
import poplib.smart_dashboard.TunableNumber;
import poplib.swerve.commands.TeleopSwerveDrive;



public class RobotContainer {
    XboxIO controller = XboxIO.getInstance();

    Flywheel flywheel = Flywheel.getInstance();
    Swerve swerve = Swerve.getInstance();
    Indexer indexer = Indexer.getInstance();
    // Turret turret = Turret.getInstance();
    Pivot pivot = Pivot.getInstance();
    // Climb climb = Climb.getInstance();
    Intake intake = Intake.getInstance();
    private final SendableChooser<Command> autoChooser;
    public static double f = ScoringSetpoints.HUB.getFlywheel();
    public static double p = ScoringSetpoints.HUB.getPivot();
    public TunableNumber rot;

    public RobotContainer() {
        autoChooser = new SendableChooser<>();
        AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Right Shoot", 
        shootRight().alongWith(new WaitCommand(10)).andThen(stopShooting()));

        autoChooser.addOption("Right Shoot + Exit", 
        shootRight().alongWith(new WaitCommand(10)).andThen(stopShooting()).andThen(swerve.moveSwerveUsingPID(0.0, -2, 0))); 
        autoChooser.addOption("Right Shoot and Move", 
        intake().andThen(shootRight()).alongWith(new WaitCommand(4)).andThen(stopShooting()).
        andThen(swerve.moveSwerveUsingPID(0, 4.1, 0)).
        andThen(new WaitCommand(3)).
        andThen(swerve.moveSwerveUsingPID(-0.2, -4.1, 0)).
        andThen(shootRight()).alongWith(new WaitCommand(4)).andThen(stopShooting()));   // 3.8608

        autoChooser.addOption("Center Shoot and Move", 
        swerve.moveSwerveUsingPID(-1, 0).
        andThen(intake.runIntake()).
        andThen(shootCenterInAuto()).alongWith(new WaitCommand(7)).andThen(stopShooting()).
        andThen(intake.stopIntake()).
        andThen(swerve.moveSwerveUsingPID(1, 0)));

        autoChooser.addOption("test center", 
        swerve.moveSwerve(-1, 0, 0, 0.2)
        );

        autoChooser.addOption("Dont Shoot", null);

        // autoChooser.addOption("use this one", intake.runIntake().
        // alongWith(intake.runSpin()).
        // andThen(swerve.moveSwerveUsingPID(0, 3.81)).
        // andThen(swerve.moveSwerveUsingPID(2.54, 0)).
        // andThen(swerve.moveSwerveUsingPID(-2.286, 0)).
        // andThen(swerve.moveSwerveUsingPID(0, 0, 180)).
        // andThen(swerve.moveSwerveUsingPID(0, -3.5052)).
        // alongWith(pivot.moveWrist(0.5*360, 0.5)).
        // andThen(flywheel.updateSetpointCommand(75, 1)).
        // andThen(indexer.runHandoff()).
        // andThen(new WaitCommand(1)).
        // andThen(indexer.runSpindexer()));

        //For auto testing DO NOT use in comp
        // autoChooser.addOption("90", swerve.moveSwerveUsingPID(0, 0, 90));
        // autoChooser.addOption("180", swerve.moveSwerveUsingPID(0, 0, 180));
        // autoChooser.addOption("270", swerve.moveSwerveUsingPID(0, 0, -90));
        // // autoChooser.addOption("rot test", swerve.moveSwerveUsingPID(0, 0, rot.get()));
        // autoChooser.addOption("drive", swerve.moveSwerveUsingPID(0, 0, 180).
        // andThen(swerve.moveSwerveUsingPID(0, 2, 0)));
        // autoChooser.addOption("Test", swerve.directionTest(-90));

        swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, controller));
        // NamedCommands.registerCommand("Extend To L1", extendToL1());
        // NamedCommands.registerCommand("Retract To Idle", extendToIdle());
        // NamedCommands.registerCommand("Shoot Fuel", shootFuel());
        // NamedCommands.registerCommand("Collect Fuel", collectFuel());
        // NamedCommands.registerCommand("Zero Pivot", pivot.reZero());
        // NamedCommands.registerCommand("Zero Turret", turret.reZero());
        SmartDashboard.putData("Auto Chooser", autoChooser);


        configureBindings();
    }

    private void configureBindings() {
        controller.getDriverTrigger(XboxController.Axis.kRightTrigger.value).onTrue(shootCenter()).onFalse(stopShooting());
        controller.getDriverTrigger(XboxController.Axis.kLeftTrigger.value).onTrue(shootFar()).onFalse(stopShooting());
        controller.getDriverButton(XboxController.Button.kX.value).onTrue(shootRight()).onFalse(stopShooting());
        controller.getDriverButton(XboxController.Button.kY.value).onTrue(intake.upIntake()).onFalse(intake.downIntake());
        // controller.getDriverButton(XboxController.Button.kB.value).onTrue(indexer.reverseHandoff().andThen(indexer.reverseSpindexer())).onFalse(indexer.stopHandoff().andThen(indexer.stopSpindexer()));
        controller.getDriverButton(XboxController.Button.kRightBumper.value).onTrue(intake());
        controller.getDriverButton(XboxController.Button.kLeftBumper.value).onTrue(stopIntaking());

        // controller.getDriverButton(XboxController.Button.kB.value).onTrue(fakeShoot());
        controller.getDriverButton(XboxController.Button.kStart.value).onTrue(swerve.resetGyroCommand());
        controller.getDriverController().povUp().onTrue(intake.runSpin());
        controller.getDriverController().povDown().onTrue(intake.stopSpin());
        controller.getDriverController().povLeft().onTrue(intake.reverseSpin()).onFalse(intake.stopSpin());
        controller.getDriverController().povRight().onTrue(intake.uppies());
        // controller.getOperatorTrigger(XboxController.Axis.kRightTrigger.value).onTrue(flywheel.updateSetpointCommand(100)).onFalse(flywheel.updateSetpointCommand(0));
        // controller.getOperatorTrigger(XboxController.Axis.kLeftTrigger.value).onTrue(flywheel.updateSetpointCommand(-100)).onFalse(flywheel.updateSetpointCommand(0));

        // controller.getOperatorTrigger(XboxController.Axis.kLeftX.value).onTrue(pivot.moveWristBy(-controller.getRawAxis(XboxController.Axis.kLeftX.value, controller.getOperatorController()), 0.1));
        // // controller.getOperatorTrigger(XboxController.Axis.kRightY.value).onTrue(turret.turnTurretBy(controller.getRawAxis(XboxController.Axis.kRightY.value, controller.getOperatorController()), 0.1));

        // controller.getOperatorButton(XboxController.Button.kX.value).onTrue(indexer.runHandoff()).onFalse(indexer.stopHandoff());
        // controller.getOperatorButton(XboxController.Button.kB.value).onTrue(indexer.reverseHandoff()).onFalse(indexer.stopHandoff());    
        // controller.getOperatorButton(XboxController.Button.kA.value).onTrue(indexer.runSpindexer()).onFalse(indexer.stopSpindexer());
        // controller.getOperatorButton(XboxController.Button.kY.value).onTrue(indexer.reverseSpindexer()).onFalse(indexer.stopSpindexer());    

        // // controller.getOperatorButton(XboxController.Button.kRightBumper.value).onTrue(intake.runIntake()).onFalse(intake.stopIntake());
        // // controller.getOperatorButton(XboxController.Button.kLeftBumper.value).onTrue(intake.reverseIntake()).onFalse(intake.stopIntake()); 

        // controller.getOperatorButton(XboxController.Button.kStart.value).onTrue(pivot.reZero().alongWith(turret.reZero()));
           

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command testCommand() {
        // return (intake()).
        // andThen(intake.runSpin()).
        // alongWith(new WaitCommand(2)).
        // andThen(intake.reverseSpin()).
        // alongWith(new WaitCommand(2)).
        // andThen(intake.stopIntake()).
        // andThen(indexer.runSpindexer()).
        // alongWith(indexer.runHandoff()).
        // alongWith(new WaitCommand(3)).
        // andThen(indexer.stopSpindexer()).
        // alongWith(indexer.stopHandoff()).
        // andThen(shootFar()).
        // alongWith(new WaitCommand(3))
        // .andThen(stopShooting());

        return (indexer.runSpindexer());
    }

    public Command shootCenter() {
        // An example command will be run in autonomous
        return pivot.moveWrist(0.4*360, 0.5). //0.5*360 before
        alongWith(flywheel.updateSetpointCommand(80, 1)). //65 before
        andThen(indexer.runHandoff()).
        andThen(new WaitCommand(0.5)).
        andThen(indexer.runSpindexer());
    }

    public Command shootPass() {
        return pivot.moveWrist(0*360, 0.5).
        alongWith(flywheel.updateSetpointCommand(30, 15)).
        alongWith(indexer.runHandoff()).
        andThen(new WaitCommand(0.5)).
        andThen(indexer.runSpindexer());
    }

    public Command shootFar() {
        return pivot.moveWrist(0.6*360, 0.5).
        alongWith(flywheel.updateSetpointCommand(45, 15)).
        alongWith(indexer.runHandoff()).
        andThen(new WaitCommand(0.5)).
        andThen(indexer.runSpindexer());
    }

        
    public Command fakeShoot() {
        return indexer.runHandoff().
        andThen(new WaitCommand(0.5)).
        andThen(indexer.runSpindexer());
    }
    
    public Command shootRight() {
        return pivot.moveWrist(0.5*360, 0.5).
        alongWith(flywheel.updateSetpointCommand(72, 1)). //prev 64
        alongWith(indexer.runHandoff()).
        andThen(new WaitCommand(0.5)).
        andThen(indexer.runSpindexer());
    }

    public Command bordieAutoRight() {
        return intake.runIntake().
        alongWith(intake.runSpin()).
        andThen(swerve.moveSwerveUsingPID(0, 3.81)).
        andThen(swerve.moveSwerveUsingPID(2.54, 0)).
        andThen(swerve.moveSwerveUsingPID(-2.286, 0)).
        andThen(swerve.moveSwerveUsingPID(0, 0, 180)).
        andThen(swerve.moveSwerveUsingPID(0, -3.5052)).
        alongWith(pivot.moveWrist(0.5*360, 0.5)).
        andThen(flywheel.updateSetpointCommand(75, 1)).
        andThen(indexer.runHandoff()).
        andThen(new WaitCommand(1)).
        andThen(indexer.runSpindexer());
    }

    public Command shootCenterInAuto() {
        return pivot.moveWrist(0.0*360, 0.5).
        alongWith(flywheel.updateSetpointCommand(55, 1)).
        alongWith(indexer.runHandoff()).
        andThen(new WaitCommand(1)).
        andThen(indexer.runSpindexer());
    }

    public Command stopShooting() {
        return pivot.moveWrist(0, 0.5).
        alongWith(flywheel.updateSetpointCommand(0, 1)).
        andThen(indexer.stopHandoff()).andThen(indexer.stopSpindexer());
    }

    public Command intake() {
        return intake.runIntake();
    }

    public Command stopIntaking() {
        return intake.stopIntake();
    }

    public Command flushRobot() {
        return indexer.reverseHandoff().
        andThen(indexer.reverseSpindexer()).
        andThen(flywheel.updateSetpointCommand(-50));
        // andThen(intake.reverseIntake());
    }

    public Command stopFlushing() {
        return indexer.stopHandoff().
        andThen(indexer.stopSpindexer()).
        andThen(flywheel.updateSetpointCommand(0));
        // andThen(intake.stopIntake());
    }

    // public Command extendToL1() {
    //     return climb.extendClimb()
    //     .andThen(() -> {StateMachine.getInstance().climb = ClimbState.EXTENDED_TO_L1;});
    // }

    // public Command extendToIdle() {
    //     return climb.unextendClimb()
    //     .andThen(() -> {StateMachine.getInstance().climb = ClimbState.IDLE;});
    // }

    public Command alignClimbLeft() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return swerve.autoAlign(CLIMB_MOVEMENT_SP.BLUE_LEFT);
            } else {
                return swerve.autoAlign(CLIMB_MOVEMENT_SP.RED_LEFT);
            }
        } else {
            return new WaitCommand(1).
            andThen(() -> DriverStation.reportWarning("CANNOT GET THE ALLIANCE, AUTOALIGN WILL NOT WORK", false));        
        }
    }

    public Command alignClimbRight() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return swerve.autoAlign(CLIMB_MOVEMENT_SP.BLUE_RIGHT);
            } else {
                return swerve.autoAlign(CLIMB_MOVEMENT_SP.RED_RIGHT);
            }
        } else {
            return new WaitCommand(1).
            andThen(() -> DriverStation.reportWarning("CANNOT GET THE ALLIANCE, AUTOALIGN WILL NOT WORK", false));
        }
    }
}
