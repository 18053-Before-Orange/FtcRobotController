package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cDeviceType
@DeviceProperties(name = "Grove Magnetic Sensor", description = "Magnetic Sensor from Grove", xmlTag = "GroveMag")
public class MagneticSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2B);

    protected MagneticSensor(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.valueOf("Grove");
    }

    @Override
    public String getDeviceName() {
        return "Grove Magnetic Sensor";
    }

    public enum Register
    {
        FIRST(0),
        DATA0_MSB(0x00),
        DATA0_LSB(0x01),
        DATA1_MSB(0x02),
        DATA1_LSB(0x03),
        DATA2_MSB(0x04),
        DATA2_LSB(0x05),
        DATA3_MSB(0x06),
        DATA3_LSB(0x07),
        RCOUNT0(0x08),
        RCOUNT1(0x09),
        RCOUNT2(0x0A),
        RCOUNT3(0x0B),
        OFFSET0(0x0C),
        OFFSET1(0x0D),
        OFFSET2(0x0E),
        OFFSET3(0x0F),
        SETTLECOUNT0(0x10),
        SETTLECOUNT1(0x11),
        SETTLECOUNT2(0x12),
        SETTLECOUNT3(0x13),
        CLOCK_DIVIDERS0(0x14),
        CLOCK_DIVIDERS1(0x15),
        CLOCK_DIVIDERS2(0x16),
        CLOCK_DIVIDERS3(0x17),
        STATUS(0x18),
        ERROR_CONFIG(0x19),
        CONFIG(0x1A),
        MUX_CONFIG(0x1B),
        RESET_DEV(0x1C),
        DRIVE_CURRENT0(0x1E),
        DRIVE_CURRENT1(0x1F),
        DRIVE_CURRENT2(0x20),
        DRIVE_CURRENT3(0x21),
        MANUFACTURER_ID(0x7E),
        DEVICE_ID(0x7F),
        RESOLUTION(0x80),

        LAST(RESOLUTION.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public void setCurrent() {
        writeShort(Register.DRIVE_CURRENT0, (short) 2);
    }

    public short getManufacturerIDRaw()
    {
        return readShort(Register.MANUFACTURER_ID);
    }

    public short getMag0LowRaw()
    {
        return readShort(Register.DATA0_LSB);
    }

    public short getMag0HighRaw()
    {
        return readShort(Register.DATA0_MSB);
    }

    public short getMag1LowRaw()
    {
        return readShort(Register.DATA1_LSB);
    }

    public short getMag1HighRaw()
    {
        return readShort(Register.DATA1_MSB);
    }

    public short getMag2LowRaw()
    {
        return readShort(Register.DATA2_LSB);
    }

    public short getMag2HighRaw()
    {
        return readShort(Register.DATA2_MSB);
    }

    public short getMag3LowRaw()
    {
        return readShort(Register.DATA3_LSB);
    }

    public short getMag3HighRaw()
    {
        return readShort(Register.DATA3_MSB);
    }

    public short getDriveCurrent0()
    {
        return readShort(Register.DRIVE_CURRENT0);
    }

    public short getDriveCurrent1()
    {
        return readShort(Register.DRIVE_CURRENT1);
    }

    public short getDriveCurrent2()
    {
        return readShort(Register.DRIVE_CURRENT2);
    }

    public short getDriveCurrent3()
    {
        return readShort(Register.DRIVE_CURRENT3);
    }

    public short getRcount0()
    {
        return readShort(Register.RCOUNT0);
    }

    public short getRcount1()
    {
        return readShort(Register.RCOUNT1);
    }

    public short getRcount2()
    {
        return readShort(Register.RCOUNT2);
    }

    public short getRcount3()
    {
        return readShort(Register.RCOUNT3);
    }

    public MagneticSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }
}
