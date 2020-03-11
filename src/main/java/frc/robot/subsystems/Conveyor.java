package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.ConveyorPeriodic;
import frc.robot.utils.ToggledBreakBeam;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.*;

public class Conveyor extends SubsystemBase {

    public CANSparkMax horizontalConveyor;
    public CANSparkMax verticalConveyor;

    private ToggledBreakBeam entryBeam;
    private ToggledBreakBeam middleBeam;
    private ToggledBreakBeam exitBeam;

    private int powerCellsStored = 0;


    public enum ConveyorState {
        SHOOTING,
        UNRESTRICTED_SHOOTING,
        INTAKING,
        EXTAKING,
        FORCED_CONVEYOR_INTAKE,
        FORCED_CONVEYOR_EXTAKE,
        DEFAULT
    }

    private ConveyorState conveyorState = ConveyorState.DEFAULT;

    private boolean enabled = false;

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

        entryBeam = new ToggledBreakBeam(new DigitalInput(4));
        middleBeam = new ToggledBreakBeam(new DigitalInput(5));
        exitBeam = new ToggledBreakBeam(new DigitalInput(6));
    }

    public void moveAll(double power) {
        horizontalConveyor.set(power);
        verticalConveyor.set(power);
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

    public void updatePowerCellCount(int ballsShot){
        powerCellsStored -= ballsShot;
    }


    public void updateConveyorMotion(boolean horiz, boolean vert, boolean reversed){
        double horizontalPower = conveyorState == ConveyorState.SHOOTING  ? 0.5 : 0.3;

        double verticalPower = conveyorState == ConveyorState.SHOOTING ? 0.6 : 0.45;

        if(conveyorState.equals(ConveyorState.UNRESTRICTED_SHOOTING)) {
            verticalPower = 1;
            horizontalPower = 1;
        }

        if (reversed) {
            horizontalPower *= -1;
            verticalPower *= -1;
        }

        if(horiz)
            horizontalConveyor.set(horizontalPower);
        else
            if (conveyorState != ConveyorState.EXTAKING)
                horizontalConveyor.set(0);

        if(vert)
            verticalConveyor.set(verticalPower);
        else
            if(conveyorState != ConveyorState.EXTAKING)
                verticalConveyor.set(0);
    }

    @Override
    public void periodic() {
        updatePowerCellCount();

        SmartDashboard.putNumber("Power Cells Stored = ", getPowerCellsStored());

        SmartDashboard.putString("Conveyor State", conveyorState.toString());
        SmartDashboard.putBoolean("Entry Beam", entryBeam.getState());
        SmartDashboard.putBoolean("Exit Beam", exitBeam.getState());
    }

    public ConveyorState getConveyorState() {
        return conveyorState;
    }

    public boolean getEntryBeam() {
        return entryBeam.getState();
    }

    public boolean getMiddleBeam() {
        return middleBeam.getState();
    }

    public boolean getExitBeam() {
        return exitBeam.getState();
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        conveyorState = ConveyorState.DEFAULT;
        moveAll(0);
    }

    public boolean enabled() {
        return enabled;
    }

    public void initDefaultCommand(BooleanSupplier shooterAtVelocity) {
        setDefaultCommand(new ConveyorPeriodic(this, shooterAtVelocity));
    }
}
