package frc.robot.commands.intake;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

import java.util.function.BooleanSupplier;

public class ConveyorPeriodic extends CommandBase {
    private BooleanSupplier shooterAtVelocity;
    private boolean horizontalBeltMovement = false;
    private boolean verticalBeltMovement = false;

    private Conveyor conveyor;

    public ConveyorPeriodic(Conveyor conveyor, BooleanSupplier shooterAtVelocity) {
        addRequirements(conveyor);
        this.shooterAtVelocity = shooterAtVelocity;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        conveyor.enable();
    }

    @Override
    public void execute() {
        if (conveyor.enabled()) {
            boolean reversed = false;

            horizontalBeltMovement = !conveyor.getEntryBeam();
            verticalBeltMovement = conveyor.getExitBeam() && horizontalBeltMovement;

            if (conveyor.getConveyorState() == Conveyor.ConveyorState.SHOOTING) {
                if (shooterAtVelocity.getAsBoolean())
                    horizontalBeltMovement = verticalBeltMovement = true;
                else
                    horizontalBeltMovement = verticalBeltMovement = false;
            }

            if (conveyor.getConveyorState() == Conveyor.ConveyorState.EXTAKING) {
                horizontalBeltMovement = true;
                verticalBeltMovement = true;
                reversed = true;
            }

            conveyor.updateConveyorMotion(horizontalBeltMovement, verticalBeltMovement, reversed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.disable();
        conveyor.moveAll(0);
    }
}
