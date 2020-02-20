package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ToggledBreakBeam;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class Conveyor extends SubsystemBase {

    public CANSparkMax horizontalConveyor;
    public CANSparkMax verticalConveyor;

    private ToggledBreakBeam entryBeam;
    private ToggledBreakBeam middleBeam;
    private ToggledBreakBeam exitBeam;

    private int powerCellsStored = 0;

    private boolean intaking = false;
    private boolean extaking = false;

    private boolean horizontalBeltMovement = false;
    private boolean verticalBeltMovement = false;

    public void setExtaking(boolean extaking) {
        this.extaking = extaking;
    }

    public enum ConveyorState {
        EMPTY,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SHOOTING,
        ERROR,
    }

    private ConveyorState conveyorState;
    private long stateTimer; // in microseconds

    private BooleanSupplier shooterAtVelocity;

    public Conveyor(BooleanSupplier shooterAtVelocity) {

        horizontalConveyor = new CANSparkMax(ConveyorConstants.kHorizontalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        verticalConveyor = new CANSparkMax(ConveyorConstants.kVerticalMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        horizontalConveyor.setSmartCurrentLimit(30);
        verticalConveyor.setSmartCurrentLimit(30);

        horizontalConveyor.setInverted(true);
        verticalConveyor.setInverted(true);

        horizontalConveyor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        verticalConveyor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        powerCellsStored = 0;
        conveyorState = ConveyorState.EMPTY;
        stateTimer = RobotController.getFPGATime();

        entryBeam = new ToggledBreakBeam(new DigitalInput(4));
        middleBeam = new ToggledBreakBeam(new DigitalInput(5));
        exitBeam = new ToggledBreakBeam(new DigitalInput(6));

        this.shooterAtVelocity = shooterAtVelocity;
    }


    public void moveAll(double power) {
        horizontalConveyor.set(power);
        verticalConveyor.set(power);
    }

    public void setIntaking(boolean intaking){
        this.intaking = intaking;
    }

    public int getPowerCellsStored() {
        return powerCellsStored;
    }

    public void setConveyorState(ConveyorState conveyorState) {
        this.conveyorState = conveyorState;
    }

    public void updatePowerCellCount(){
        entryBeam.update();
        exitBeam.update();
        if (entryBeam.getToggled()) powerCellsStored++;
        if (exitBeam.getToggled()) powerCellsStored--;
    }

    public void updateEnum(){

        if(getPowerCellsStored() > 5 || getPowerCellsStored() < 0)
            conveyorState = ConveyorState.ERROR;
        else
            conveyorState = ConveyorState.values()[getPowerCellsStored()];

        /*switch (getPowerCellsStored()){
            case 0:
                setConveyorState(Conveyor.ConveyorState.EMPTY);
                break;
            case 1:
                setConveyorState(Conveyor.ConveyorState.ONE);
                break;
            case 2:
                setConveyorState(Conveyor.ConveyorState.TWO);
                break;
            case 3:
                setConveyorState(Conveyor.ConveyorState.THREE);
                break;
            case 4:
                setConveyorState(Conveyor.ConveyorState.FOUR);
                break;
            case 5:
                setConveyorState(Conveyor.ConveyorState.FIVE);
                break;
            default:
                setConveyorState(Conveyor.ConveyorState.ERROR);
                break;
        }*/
    }

    public void decrementIntakeCount(){
        if (entryBeam.getToggled()) powerCellsStored--;
    }

    public void updateConveyorMotion(){
        if(horizontalBeltMovement)
            horizontalConveyor.set(0.3);
        else
            if(!extaking)
                horizontalConveyor.set(0);

        if(verticalBeltMovement)
            verticalConveyor.set(0.2);
        else
            if(!extaking)
                verticalConveyor.set(0);
    }

    @Override
    public void periodic() {
        updatePowerCellCount();
        updateEnum();

        SmartDashboard.putNumber("Power Cells Stored = ", getPowerCellsStored());

//        switch (conveyorState) {
//            case EMPTY:
//                horizontalBeltMovement = false;
//                verticalBeltMovement = false;
//
//                if(intaking)
//                    horizontalBeltMovement = true;
//
//                if (entryBeam.getToggled()) conveyorState = ConveyorState.ONE;
//                break;
//
//            case ONE:
//                horizontalBeltMovement = middleBeam.getState();
//                verticalBeltMovement = false;
//
//                if(intaking)
//                    horizontalBeltMovement = true;
//
//                if (entryBeam.getToggled()) {
//                    conveyorState = ConveyorState.TWO;
//                    stateTimer = RobotController.getFPGATime();
//                }
//                break;
//
//            case TWO:
//                horizontalBeltMovement = verticalBeltMovement = exitBeam.getState();
//
//                if(intaking)
//                    horizontalBeltMovement = true;
//
//                if (entryBeam.getToggled()) conveyorState = ConveyorState.THREE;
//
//                break;
//
//            case THREE:
//                verticalBeltMovement = exitBeam.getState();
//                if(exitBeam.getState())
//                    horizontalBeltMovement = true;
//                else
//                    horizontalBeltMovement = middleBeam.getState();
//
//                if(intaking)
//                    horizontalBeltMovement = true;
//
//                if (entryBeam.getToggled()) {
//                    conveyorState = ConveyorState.FOUR;
//                    stateTimer = RobotController.getFPGATime();
//                }
//                break;
//
//            case FOUR:
//                verticalBeltMovement = exitBeam.getState();
//                horizontalBeltMovement = !entryBeam.getState();
//
//                if(powerCellsStored < 4 && intaking)
//                    horizontalBeltMovement = true;
//                else
//                    horizontalBeltMovement = false;
//                break;
//
//            case FIVE:
//                verticalBeltMovement = exitBeam.getState();
//                if (RobotController.getFPGATime() < (stateTimer + 1e6)) horizontalBeltMovement = true;
//                else horizontalBeltMovement = false;
//                break;
//
//            case SHOOTING:
////                horizontalBeltMovement = verticalBeltMovement = shooterAtVelocity.getAsBoolean();
////                if(!verticalBeltMovement)
////                    verticalBeltMovement = exitBeam.getState();
//                horizontalBeltMovement = verticalBeltMovement = true;
//                break;
//
//        }

        horizontalBeltMovement = verticalBeltMovement = !entryBeam.getState();
        if(!exitBeam.getState())
            verticalBeltMovement = false;

        updateConveyorMotion();

        SmartDashboard.putString("Conveyor State", conveyorState.toString());

        SmartDashboard.putBoolean("Horizontal Belts Moving", horizontalBeltMovement);
        SmartDashboard.putBoolean("Vertical Belts Moving", verticalBeltMovement);

        SmartDashboard.putBoolean("Entry Beam", entryBeam.getState());
        SmartDashboard.putBoolean("Middle Beam", middleBeam.getState());
        SmartDashboard.putBoolean("Exit Beam", exitBeam.getState());


    }
}
