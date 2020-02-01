package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.utils.ToggledBreakBeam;

public class Conveyor extends SubsystemBase {

    private CANSparkMax horizontalConveyor;
    private CANSparkMax verticalConveyor;

    private ToggledBreakBeam entryBeam;

    private int powerCellsStored = 0;

    public Conveyor() {

        horizontalConveyor = new CANSparkMax(ConveyorConstants.kHorizontalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        verticalConveyor = new CANSparkMax(ConveyorConstants.kVerticalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        horizontalConveyor.setSmartCurrentLimit(30);
        verticalConveyor.setSmartCurrentLimit(30);

        horizontalConveyor.setInverted(false);
        verticalConveyor.setInverted(false);

        powerCellsStored = 0;

        entryBeam = new ToggledBreakBeam(new DigitalInput(0));
    }


    public void moveAll(double power) {
        horizontalConveyor.set(power);
        verticalConveyor.set(power);
    }

    public int getPowerCellsStored() {
        return powerCellsStored;
    }

    @Override
    public void periodic() {
        entryBeam.update();

        if (entryBeam.getToggled()) powerCellsStored++;
    }
}
