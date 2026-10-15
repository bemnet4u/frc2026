// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package poplib.subsystems.pivot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import poplib.control.FFConfig;
import poplib.control.PIDConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.smart_dashboard.PIDTuning;

public class TalonPivot extends Pivot {
    public final TalonFX leadMotor;
    // protected TalonFX followerMotor;
    protected PIDTuning pid;
    protected PositionDutyCycle position;
    protected DigitalInput limitSwitch;
    protected boolean usePID;

    public TalonPivot(MotorConfig leadConfig, FollowerConfig followerConfig, int limitSwitchID, double gearRatio, FFConfig ffConfig, boolean tuningMode, String subsystemName) {
        super(ffConfig, gearRatio, tuningMode, subsystemName);
        leadMotor = leadConfig.createTalon();
        // if (followerConfig != null) {
        //     followerMotor = followerConfig.createTalon();
        // } else {
        //     followerMotor = null;
        // };
        limitSwitch = new DigitalInput(limitSwitchID);
        pid = new PIDTuning(subsystemName, PIDConfig.getZeroPid(), tuningMode);
        position = new PositionDutyCycle(0.0);
        position.withSlot(leadMotor.getClosedLoopSlot().getValue());
    }

    public TalonPivot(MotorConfig leadConfig, int limitSwitchID, double gearRatio, FFConfig ffConfig, boolean tuningMode, String subsystemName) {
        super(ffConfig, gearRatio, tuningMode, subsystemName);
        leadMotor = leadConfig.createTalon();
        // followerMotor = null;
        usePID = true;
        limitSwitch = new DigitalInput(limitSwitchID);
        // leadMotor.setPosition(0);
        pid = leadConfig.genPIDTuning("Pivot Motor " + subsystemName, tuningMode);
        position = new PositionDutyCycle(0.0);
        position.withSlot(leadMotor.getClosedLoopSlot().getValue());
    }

    @Override
    public boolean atSetpoint(double error, double setpoint) {
        return getError(setpoint) < error;
    }

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getPosition().getValueAsDouble() - setpoint);
    }

    public void moveToZero(double speed) {
        usePID = false;
        leadMotor.set(-speed);
    }

    public void stopAndZero() {
        usePID = false;
        leadMotor.set(0);
        leadMotor.setPosition(0);
        usePID = true;
    }

    public boolean isLimitSwitchPressed() {
        return limitSwitch.get();
    }

    @Override
    public void log() {
        super.log();
        SmartDashboard.putNumber(getName() + " Position", leadMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Speed", leadMotor.get());
        SmartDashboard.putNumber(getName() + " Voltage", leadMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean(getName() + " UsePID", usePID);
    }

    @Override
    public void periodic() {
        log();
        if (usePID) {
            pid.updatePID(leadMotor);
            leadMotor.setControl(position.withPosition(setpoint.get()).withFeedForward((ff.getKg() > 0 && setpoint.get() >= 2) ? ff.calculate(Math.toRadians(0), 0) : 0));
        }
    }

    public Command reZero() {
        return runOnce(() -> {moveToZero(0.1);}).
        until(this::isLimitSwitchPressed).
        andThen(() -> {stopAndZero();});
    }
}