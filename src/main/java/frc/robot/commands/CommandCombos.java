package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;

public final class CommandCombos {

    public static Command moveElevatorAndPlace(Elevator elevator, Gripper gripper, Flipper flipper) {
        return elevator.goToDesiredHeight()
                .andThen(gripper.ungrip().withTimeout(1.0), flipper.flip())
                .finallyDo((interrupted) -> elevator.setPowerZero());
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
