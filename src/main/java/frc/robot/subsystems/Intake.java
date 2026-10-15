// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import poplib.subsystems.pivot.TalonPivot;

public class Intake extends TalonPivot {
    private static Intake instance;
    private final TalonFX leadSpinMotor;
    @SuppressWarnings("unused")
    private final TalonFX followerMotor;
    private final CANdi limitSwitchKinda;
    private double timer;
    private double DOWN;
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private Intake() {
        super(
            Constants.Intake.PIVOT_CONFIG, 
            7, 
            1, 
            Constants.Intake.FF, 
            Constants.Intake.TUNING_MODE, 
            "Intake"
        );
        leadSpinMotor = Constants.Intake.MOTOR_CONFIG.createTalon();      //roller
        followerMotor = Constants.Intake.FOLLOWER_CONFIG.createTalon();   //roller
        super.leadMotor.setPosition(0);    //pivot
        limitSwitchKinda = new CANdi(23, "hi");
        DOWN = Constants.Intake.DOWN/360.0;
        timer = 75; //failsafe for intake incase limit switch no work
    }

    public Command dropDozer() {
        return moveWrist(Constants.Intake.DOWN, 0.5);
    }

    public Command upDozer() {
        return moveWrist(0, 0.5);
    }


    public Command runIntakeAuto() {
        return moveWrist(Constants.Intake.DOWN, 10).andThen(runOnce(() -> leadSpinMotor.set(Constants.Intake.SPEED)));
    }

    // Do as i say, not as i do
    public double getIntakeMotorState() {
        return leadSpinMotor.get();
    }

    /**
     * Runs the intake :shock:
     * @return the Command that runs the intake
     */
    public Command runIntake() {
        return moveWrist(DOWN*360, 3).
        andThen(runOnce(() -> timer = 75)).
        andThen(dropALil());
    }

    public Command dropALil() {
        return run(() -> {usePID = false; leadMotor.set(0.03); timer--;}).
        onlyWhile(() -> {return !limitSwitchKinda.getS1Closed().getValue() && timer > 0;}).
        andThen(runOnce(() -> {leadMotor.set(0); DOWN = leadMotor.getPosition().getValueAsDouble();})).
        andThen(runOnce(() -> setpoint.setDefault(DOWN))).
        andThen(runOnce(() -> {usePID = true;}));
    }

    public Command upIntake() { //for wiggle
        return runOnce(() -> {usePID = true; setpoint.setDefault(DOWN-7);});
    }

    public Command downIntake() { //for wiggle
        return runOnce(() -> setpoint.setDefault(DOWN));
    }

    public Command runSpin() {
        return runOnce(() -> leadSpinMotor.set(Constants.Intake.SPEED));
    }
    
    public Command stopSpin() {
        return runOnce(() -> leadSpinMotor.set(0));
    }

    public Command uppies() {
        return runOnce(() -> setpoint.setDefault(DOWN - 1));
    }

    public Command reverseSpin() {
        return runOnce(() -> leadSpinMotor.set(-Constants.Intake.SPEED));
    }

    /**
     * Stops the indexer
     * @return the Command that stops the indexer
     */
    public Command stopIntake() {
        return runOnce(() -> usePID = true).andThen(moveWrist(0, 10)).andThen(runOnce(() -> leadSpinMotor.set(0)));
    }

    /**
     * Runs the indexer on reverse
     * @return the Command that runs the indexer on reverse
     */
    public Command reverseIntake() {
        return moveWrist(Constants.Intake.DOWN, 10).andThen(runOnce(() -> leadSpinMotor.set(-Constants.Intake.SPEED)));
    }
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Motor Speed", leadSpinMotor.get());
        SmartDashboard.putBoolean("Intake Limit", limitSwitchKinda.getS1Closed().getValue());
        SmartDashboard.putNumber("Theo Down", DOWN);
        SmartDashboard.putNumber("Intake Timer", timer);
    }
}