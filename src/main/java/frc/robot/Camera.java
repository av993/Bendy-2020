package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera
{
    NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
     
    public Camera()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    public double getXOffset()
    {
        return -tx.getDouble(0.0);
    }

    public double getYOffset()
    {
        return ty.getDouble(0.0);
    }

    public double getDistance(){
        double h2 = 98.25; // height of the middle of the pp
        double h1 = 25; // height from the ground to the limelight which is seeking the pp
        double a2 = getYOffset(); // angle from the limelight to the pp
        double a1 = 0; // incline of the limelight

        double distance = (h2-h1) / Math.tan(Math.toRadians(a1 + a2)); // distance to get to the pp
        return distance;
    }

    public double getArea()
    {
        return ta.getDouble(0.0);
    }

    public boolean hasValid()
    {
        if(tv.getDouble(0.0)<1)return false;
        return true;
    }
}

