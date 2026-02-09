package frc.robot.subsystems.drive;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import org.lasarobotics.fsm.StateMachine;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    private final MAXSwerve swerveDrive;

    public enum DriveSubsystemStates implements SystemState {
        TELEOP {
            @Override
            public void execute() {
                getInstance().drive();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static DriveSubsystem s_driveSubsystemInstance;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_rightX;
    private BooleanSupplier m_reset;

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.TELEOP);
        swerveDrive = new MAXSwerve();
    }

    public void configureBindings(
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX,
            BooleanSupplier reset) {
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_reset = reset;
    }

    public void drive() {
        
        swerveDrive.drive(
            m_leftX.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_leftY.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_rightX.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    }

    public void drive(double leftX, double leftY, double rightX) {
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/RightX", m_rightX.getAsDouble());
        
        swerveDrive.drive(
            leftX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            leftY * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            rightX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    } 

    public void periodic() {
        if (m_reset.getAsBoolean())
            zeroGyro();
        swerveDrive.periodic();
        Logger.recordOutput("DriveSubsystem/pose", swerveDrive.getPose());
        Logger.recordOutput("DriveSubsystem/heading", swerveDrive.getHeading());
        Logger.recordOutput(getName() + "/state", getState().toString());
    }

    public void zeroGyro() {
        swerveDrive.zeroHeading();
    }

    public void offset180() {
        swerveDrive.offset180();
    }

    public void close() {}
}

//henry wuz here