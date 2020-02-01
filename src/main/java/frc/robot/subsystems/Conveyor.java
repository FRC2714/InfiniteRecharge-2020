package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.utils.ToggledBreakBeam;

public class Conveyor extends SubsystemBase {

    private CANSparkMax horizontalConveyor;
    private CANSparkMax verticalConveyor;

    private ToggledBreakBeam entryBeam;
    private ToggledBreakBeam middleBeam;
    private ToggledBreakBeam exitBeam;

    private int powerCellsStored = 0;

    private boolean intakeState = false;

    private boolean horizontalBeltMovement = false;
    private boolean verticalBeltMovement = false;

    public Conveyor() {

//        horizontalConveyor = new CANSparkMax(ConveyorConstants.kHorizontalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
//        verticalConveyor = new CANSparkMax(ConveyorConstants.kVerticalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

//        horizontalConveyor.setSmartCurrentLimit(30);
//        verticalConveyor.setSmartCurrentLimit(30);
//
//        horizontalConveyor.setInverted(false);
//        verticalConveyor.setInverted(false);

        powerCellsStored = 0;

        entryBeam = new ToggledBreakBeam(new DigitalInput(4));
        middleBeam = new ToggledBreakBeam(new DigitalInput(5));
        exitBeam = new ToggledBreakBeam(new DigitalInput(6));
    }


    public void moveAll(double power) {
        horizontalConveyor.set(power);
        verticalConveyor.set(power);
    }

    public void setIntakeState(boolean intakeState){
        this.intakeState = intakeState;
    }

    public int getPowerCellsStored() {
        return powerCellsStored;
    }

    @Override
    public void periodic() {
        entryBeam.update();
        if (entryBeam.getToggled()) powerCellsStored++;
        SmartDashboard.putNumber("Power Cells Stored = ", getPowerCellsStored());

        if(powerCellsStored > 0 && powerCellsStored <= 2 && middleBeam.getState()) {
            horizontalBeltMovement = true;
            verticalBeltMovement = false;
        } else if (powerCellsStored > 2 && powerCellsStored <= 5 && exitBeam.getState()){
            horizontalBeltMovement = true;
            verticalBeltMovement = true;
        } else if (powerCellsStored <= 5 && !exitBeam.getState() ) {
            horizontalBeltMovement = true;
            verticalBeltMovement = false;
        } else {
            horizontalBeltMovement = false;
            verticalBeltMovement = false;
        }

        SmartDashboard.putBoolean("Horizontal Belts Moving", horizontalBeltMovement);
        SmartDashboard.putBoolean("Vertical Belts Moving", verticalBeltMovement);

        SmartDashboard.putBoolean("Entry Beam", entryBeam.getState());
        SmartDashboard.putBoolean("Middle Beam", middleBeam.getState());
        SmartDashboard.putBoolean("Exit Beam", exitBeam.getState());


    }
}
