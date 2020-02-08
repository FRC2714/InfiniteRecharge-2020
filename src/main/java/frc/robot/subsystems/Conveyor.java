package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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

    public enum ConveyorState {
        EMPTY,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SHOOTING
    }

    private ConveyorState conveyorState;
    private long stateTimer; // in microseconds

    public Conveyor() {

//        horizontalConveyor = new CANSparkMax(ConveyorConstants.kHorizontalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
//        verticalConveyor = new CANSparkMax(ConveyorConstants.kVerticalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

//        horizontalConveyor.setSmartCurrentLimit(30);
//        verticalConveyor.setSmartCurrentLimit(30);
//
//        horizontalConveyor.setInverted(false);
//        verticalConveyor.setInverted(false);

        powerCellsStored = 0;
        conveyorState = ConveyorState.EMPTY;
        stateTimer = RobotController.getFPGATime();

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

    public void setConveyorState(ConveyorState conveyorState) {
        this.conveyorState = conveyorState;
    }

    @Override
    public void periodic() {
        entryBeam.update();
        if (entryBeam.getToggled()) powerCellsStored++;

        SmartDashboard.putNumber("Power Cells Stored = ", getPowerCellsStored());


//        if shooting: conveyor state = shooting;

        switch (conveyorState) {
            case EMPTY:
                horizontalBeltMovement = false;
                verticalBeltMovement = false;

                if(intakeState){
                    horizontalBeltMovement = true;
                    verticalBeltMovement = true;
                }

                if (entryBeam.getToggled()) conveyorState = ConveyorState.ONE;
                break;

            case ONE:
                horizontalBeltMovement = middleBeam.getState();
                verticalBeltMovement = false;

                if (entryBeam.getToggled()) {
                    conveyorState = ConveyorState.TWO;
                    stateTimer = RobotController.getFPGATime();
                }
                break;

            case TWO:
                if (RobotController.getFPGATime() < (stateTimer + 2e6)) {
                    horizontalBeltMovement = true;
                    verticalBeltMovement = false;
                } else if (exitBeam.getState()) {
                    horizontalBeltMovement = true;
                    verticalBeltMovement = true;
                } else {
                    horizontalBeltMovement = false;
                    verticalBeltMovement = false;
                }

                if (entryBeam.getToggled()) conveyorState = ConveyorState.THREE;
                break;

            case THREE:
                verticalBeltMovement = exitBeam.getState();
                horizontalBeltMovement = middleBeam.getState();

                if (entryBeam.getToggled()) {
                    conveyorState = ConveyorState.FOUR;
                    stateTimer = RobotController.getFPGATime();
                }
                break;

            case FOUR:
                verticalBeltMovement = exitBeam.getState();

                if (RobotController.getFPGATime() < (stateTimer + 2e6)) horizontalBeltMovement = true;
                else horizontalBeltMovement = false;

                if (entryBeam.getToggled()) {
                    conveyorState = ConveyorState.FIVE;
                    stateTimer = RobotController.getFPGATime();
                }
                break;

            case FIVE:
                verticalBeltMovement = exitBeam.getState();

                if (RobotController.getFPGATime() < (stateTimer + 1e6)) horizontalBeltMovement = true;
                else horizontalBeltMovement = false;

                break;

            case SHOOTING:
                if (!exitBeam.getState()) {
                    horizontalBeltMovement = true;
                    verticalBeltMovement = true;
                } else {
                    horizontalBeltMovement = false;
                    verticalBeltMovement = false;
            }
                break;
        }

        SmartDashboard.putString("Conveyor State", conveyorState.toString());

        SmartDashboard.putBoolean("Horizontal Belts Moving", horizontalBeltMovement);
        SmartDashboard.putBoolean("Vertical Belts Moving", verticalBeltMovement);

        SmartDashboard.putBoolean("Entry Beam", entryBeam.getState());
        SmartDashboard.putBoolean("Middle Beam", middleBeam.getState());
        SmartDashboard.putBoolean("Exit Beam", exitBeam.getState());


    }
}
