package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Constants.RobotType;
import frc.robot.RobotContainer.TGR;

public class Elevator extends SubsystemBase {
    TalonFX elevatorMotor;
    boolean testing = true;
    Height targetHeight = Height.OFF;

    final static boolean elevatorInverted = (Constants.ROBOTTYPE == RobotType.PBOT) ? false : true;

    public Elevator() {
        elevatorMotor = new TalonFX(14);
        var talonFXConfigurator = elevatorMotor.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();
        var motionMagicConfigs = new MotionMagicConfigs();
        var slotConfigs = new SlotConfigs();
        
        motorConfigs.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        motionMagicConfigs.withMotionMagicCruiseVelocity(ELEVATOR.kElevatorCruiseVelocity).withMotionMagicAcceleration(ELEVATOR.kElevatorAcceleration);
        slotConfigs.withKP(0.05).withKI(0).withKD(0.5);

        talonFXConfigurator.apply(motorConfigs);
        talonFXConfigurator.apply(motionMagicConfigs);
        talonFXConfigurator.apply(slotConfigs);
    }

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Elevator", getPosition());
        SmartDashboard.putBoolean("Elevator At Position", isCorrectElevatorHeight());
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public Command goToDesiredHeight() {
        if (testing)
            return Commands.none();
        return goToDesiredHeight(getDesiredHeight());
    }

    public Command goToDesiredHeight(Height height) {
        if (testing)
            return Commands.none();
        Command command = runOnce(() -> targetHeight = height);

        if (height == Height.OFF)
            return command.andThen(runOnce(() -> setPowerZero()));
        return command.andThen(runOnce(() -> changeHeight(height)),
                Commands.waitUntil(() -> isCorrectElevatorHeight()));
    }

    public void setPowerZero() {
        elevatorMotor.set(0);
    }

    public Height getDesiredHeight() {
        Height desiredHeight = Height.LOW;
        if (TGR.PlaceHigh.bool() && TGR.PlaceMid.bool()) {
            desiredHeight = Height.LOW;
        } else if (TGR.PlaceHigh.bool())
            desiredHeight = Height.HIGH;
        else if (TGR.PlaceMid.bool())
            desiredHeight = Height.MID;
        return desiredHeight;
    }

    private void changeHeight(ELEVATOR.Height height) {
        elevatorMotor.setControl(new MotionMagicDutyCycle(height.height));
    }

    private boolean isCorrectElevatorHeight() {
        return Math.abs(targetHeight.height - getPosition()) < 4000;
    }

    public Command elevatorVoltage(double percent) {
        return startEnd(() -> elevatorMotor.set(percent), () -> elevatorMotor.set(0));
    }
}
