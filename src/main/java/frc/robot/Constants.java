// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import poplib.control.FFConfig;
import poplib.control.PIDConfig;
import poplib.motor.ConversionConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.Mode;
import poplib.motor.MotorConfig;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.StdDevStrategy;
import poplib.swerve.swerve_constants.SDSModules;
import poplib.swerve.swerve_constants.SwerveModuleConstants;



public final class Constants {

    public static final String swerveLoop = "cantBUS";
    public static final String manipulatorLoop = "rio"; 

    public static class Swerve {
        static final SDSModules MODULE_TYPE = SDSModules.MK4nL2;
        static final boolean TUNING_MODE = false;
        static final int SWERVE_CAN_ID_OFFSET = 5;      

        static final MotorConfig DRIVE_CONFIG = new MotorConfig(swerveLoop, 60, false, PIDConfig.getPid(0.01, 0.25), Mode.BRAKE);
        static final MotorConfig ANGLE_CONFIG = new MotorConfig(swerveLoop, 40, false, PIDConfig.getPid(5.0), Mode.BRAKE);
        
        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
            new Rotation2d[] {
                Rotation2d.fromDegrees(7.119141),              // set offsets
                Rotation2d.fromDegrees(99.755859),           // set offsets
                Rotation2d.fromDegrees(142.734375),             // set offsets
                Rotation2d.fromDegrees(289.248047)              //  set offsets
            },
            MODULE_TYPE, 
            TUNING_MODE, 
            DRIVE_CONFIG,
            ANGLE_CONFIG,
            SWERVE_CAN_ID_OFFSET
        );

        public static final int PIGEON_ID = 19;
        public static final boolean GYRO_INVERSION = false;      // change if needed - gyro should be ccw+ and cw-

        public static final double WHEEL_BASE =  edu.wpi.first.math.util.Units.inchesToMeters(22);
        public static final double TRACK_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(22); 
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );
    }

    public static class Intake {
        public static final MotorConfig MOTOR_CONFIG = new MotorConfig(20, "hi", 30, true, Mode.COAST);
        public static final FollowerConfig FOLLOWER_CONFIG = new FollowerConfig(MOTOR_CONFIG, true, 21);
        public static final double SPEED = 1;
        public static final boolean TUNING_MODE = false;
        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(22, "hi", 30, false, new PIDConfig(0.05, 0.0, 0.0), Mode.BRAKE);
        // public static final MotorConfig INTAKE_GUIDE_MOTOR = new MotorConfig(23, "hi", 25, false, Mode.COAST);
        public static final double DOWN = 15*360.0;
        public static final FFConfig FF = new FFConfig(0.0);
    }

    public static class Indexer {
        public static final MotorConfig MOTOR_CONFIG = new MotorConfig(30, swerveLoop, 50, false, Mode.COAST);
        public static final MotorConfig HANDOFF_CONFIG = new MotorConfig(31, manipulatorLoop, 60, true, Mode.COAST);
        public static final int BEAMBREAK_ID = 0;
        public static final double SPEED = 1; // adjust as necessary
    }

    public static class Turret {
        public static final MotorConfig ROT_CONFIG = new MotorConfig(40, manipulatorLoop, 25, false, PIDConfig.getPid(0.1, 0, 0, 0), Mode.BRAKE);
        public static final int LIMIT_SWITCH_ID = 2;
        public static final int GEAR_RATIO = 100; //check
        public static final FFConfig FF_CONFIG = new FFConfig(0, 0, 0);
    }

    public static class Pivot {
        public static final boolean TUNING_MODE = false;
        public static final MotorConfig PIVOT_MOTOR = new MotorConfig(41, manipulatorLoop, 25, false, new PIDConfig(0.5, 0, 0), Mode.BRAKE, new ConversionConfig());
        public static final int LIMIT_SWITCH_ID = 1;  // 0 hub, 3 pass
        public static final int CANDI_ID = 44;
        public static final double GEAR_RATIO = 17572.0 / 336.0;
        public static final FFConfig FF = new FFConfig(0);
        public static final double MIN_ANGLE = 55.55; 
        public static final double MAX_ANGLE = 78.55;
    }

    public static class Flywheel {
        public static final boolean TUNING_MODE = false;   // 52 hub, 95 pass
        public static final MotorConfig leadConfig = new MotorConfig(42, manipulatorLoop, 60, false, new PIDConfig(0.1, 0, 0.01), Mode.BRAKE, new ConversionConfig());
        public static final FollowerConfig followConfig = new FollowerConfig(leadConfig, true, 43);
    }

    public static enum ScoringSetpoints {
        PASS(3*360, 105),
        HUB(0, 58);

        private double pivot;
        private double flywheel;

        private ScoringSetpoints(double pivot, double flywheel) {
            this.pivot = pivot;
            this.flywheel = flywheel;
        }

        public double getPivot() {
            return this.pivot;
        }

        public double getFlywheel() {
            return this.flywheel;
        }
    }

    public static class Climb {
        public static final boolean TUNING_MODE = false;
        public enum CLIMB_SETPOINT {
            IDLE(2*360),
            L15(32*360),
            L1(74*360);

            private double climb;

            private CLIMB_SETPOINT(double climb) {
            this.climb = climb;
            }
            public double getSetpoint() {
                return climb;
            }
        }
        public static final MotorConfig MOTOR_CONFIG = new MotorConfig(50, swerveLoop, 40, true, new PIDConfig(0.1, 0, 0), Mode.BRAKE);
    }


    public static class Autos {
        public static PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
            new PIDConstants(5, 0, 0), 
            new PIDConstants(5, 0, 0)
        );
        public static RobotConfig getConfig() {
            RobotConfig config = null;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
            return config;
        } 
    }

    public static class AutoAlign {
        // public static final CameraConfig alignConfig = new CameraConfig("climbCamera", new Transform3d(), 0, 0, StdDevStrategy.AMBIGUITY, AprilTagFields.k2026RebuiltWelded);
        
        public enum CLIMB_MOVEMENT_SP {
            BLUE_LEFT(0.321, 31),
            BLUE_RIGHT(-.321, 31),
            RED_LEFT(0.321, 15),
            RED_RIGHT(-0.321, 15);
            private double y;
            private int tag;

            private CLIMB_MOVEMENT_SP(double y, int tag) {
            this.y = y;
            this.tag = tag;
            }
            public double getY() {
                return y;
            }
            public double getRot() {
                return -90;
            }
            public int getTag() {
                return tag;
            }
        }
    }
}
