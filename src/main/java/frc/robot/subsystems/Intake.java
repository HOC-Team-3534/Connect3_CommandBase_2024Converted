package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Intake extends SubsystemBase {
    TalonFX topMotor, botMotor;
    boolean testing = false;

    public Intake() {
        if (!testing) {
            topMotor = new TalonFX(15);
            botMotor = new TalonFX(16);
            topMotor.getConfigurator().apply(new TalonFXConfiguration());
            botMotor.getConfigurator().apply(new TalonFXConfiguration());
            topMotor.setInverted(true);
            botMotor.setInverted(true);
        }
    }

    public Command runIntake() {
        if (testing)
            return Commands.none();
        return runEnd(() -> {
            // TODO determine power outputs for intake for best results
            if (TGR.Intake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(0.3);
                else
                    setBothMotors(0.85);
            else if (TGR.Extake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(-0.3);
                else
                    setBothMotors(-0.8);
            else
                setBothMotors(0);
        }, () -> setBothMotors(0));
    }

    public Command runIntakeAuton() {
        if (testing)
            return Commands.none();
        return startEnd(() -> setBothMotors(0.3), () -> setBothMotors(0));
    }

    private void setBothMotors(double percent) {
        topMotor.set(percent);
        botMotor.set(percent);
    }
}
