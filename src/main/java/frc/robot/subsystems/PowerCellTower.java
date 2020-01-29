package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ToggledBreakBeam;

public class PowerCellTower extends SubsystemBase {

    private CANSparkMax closeToIntakeMotor;
    private CANSparkMax midSectionMotor1;
    private CANSparkMax midSectionMotor2;
    private CANSparkMax shooterFeederMotor;

    private ToggledBreakBeam entryBeam;

    private int powerCellsStored = 0;

    public PowerCellTower() {

//        closeToIntakeMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
//        midSectionMotor1 = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
//        midSectionMotor2 = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
//        shooterFeederMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

        closeToIntakeMotor.setSmartCurrentLimit(30);
        midSectionMotor1.setSmartCurrentLimit(30);
        midSectionMotor2.setSmartCurrentLimit(30);
        shooterFeederMotor.setSmartCurrentLimit(30);

        closeToIntakeMotor.setInverted(false);
        midSectionMotor1.setInverted(false);
        midSectionMotor2.setInverted(false);
        shooterFeederMotor.setInverted(false);

        powerCellsStored = 0;

        entryBeam = new ToggledBreakBeam(new DigitalInput(0));

    }


    public void powerAll(double power) {
        closeToIntakeMotor.set(power);
        midSectionMotor1.set(power);
        midSectionMotor2.set(power);
        shooterFeederMotor.set(power);
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
