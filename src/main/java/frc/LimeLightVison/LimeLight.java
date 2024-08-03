package frc.LimeLightVison;

import java.sql.Time;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.LimeLightVison.ControlMode;
import frc.LimeLightVison.ControlMode.Advanced_Crosshair;
import frc.LimeLightVison.ControlMode.Advanced_Target;
import frc.LimeLightVison.ControlMode.CamMode;


//Limelight setup (network tables and targeting)
//do i know how this works...no
public class LimeLight {
    

    private NetworkTable m_table;
    private String m_tableName;

    public LimeLight(){
        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    }


    public LimeLight(String tableName) {
        m_tableName = tableName;
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    }

    public LimeLight(NetworkTable table){
        m_table = table;
    }


    private void testAllTab(){
        ShuffleboardTab LimeLightTab = Shuffleboard.getTab(m_tableName);
    }

    private boolean isConnected(){
        resetPipelineLatency();
        Timer.delay(.5);

        if(getPipelineLatency() == 0.0){
            return false;
        } else{
            return true;
        }
    }

    //checks validity of target
    public boolean getIsTargetFound(){
        NetworkTableEntry tv = m_table.getEntry("tv");
        double v = tv.getDouble(0);
        if(v == 0.0f){
            return false;
        } else{
            return true;
        }
    }

    public double getdegRotationToTarget(){
        NetworkTableEntry tx = m_table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    public double getdegVerticalToTarget(){
        NetworkTableEntry ty = m_table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }

    public double getTargetArea(){
        NetworkTableEntry ta = m_table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }


    public double getSkew_Rotation(){
        NetworkTableEntry ts = m_table.getEntry("ts");
        double s = ts.getDouble(0.0);
        return s;
    }

    public double getPipelineLatency(){
        NetworkTableEntry tl = m_table.getEntry(("tl"));
        double l = tl.getDouble(0.0);
        return l;
    }

    public void resetPipelineLatency(){
        m_table.getEntry("tl").setValue(0.0);
    }


    public  void setCamMode(CamMode camMode){
        m_table.getEntry("camMode").setValue(camMode.getValue());
    }


    public CamMode getCamMode(){
        NetworkTableEntry camMode = m_table.getEntry(("camMode"));
        double cam = camMode.getDouble(0.0);
        CamMode mode = CamMode.getByValue(cam);
        return mode;
    }


    public void setPipeline(Integer pipeline){
        if(pipeline < 0){
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        } else if(pipeline>9){
            pipeline = 9;
            throw new IllegalArgumentException("Pipelien can not be greater than nine");
        }
        m_table.getEntry("pipline").setValue(pipeline);
    }

    public double getPipeline(){
        NetworkTableEntry pipeline = m_table.getEntry("pipeline");
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }


    public Integer getPipelineInt(){
        NetworkTableEntry pipeline = m_table.getEntry("pipeline");
        Integer pipe = (int) pipeline.getDouble(0.0);
        return pipe;
    }

    public double getAdvanced_RotationToTarget(Advanced_Target raw){
        NetworkTableEntry txRaw = m_table.getEntry("tx" + Integer.toString(raw.getValue()));
        double x = txRaw.getDouble(0.0);
        return x;
    }

    public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
        NetworkTableEntry tyRaw = m_table.getEntry("ty" + Integer.toString(raw.getValue()));
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    public double getAdvanced_TargetArea(Advanced_Target raw) {
        NetworkTableEntry taRaw = m_table.getEntry("ta" + Integer.toString(raw.getValue()));
        double a = taRaw.getDouble(0.0);
        return a;
    }
    
    public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
        NetworkTableEntry tsRaw = m_table.getEntry("ts" + Integer.toString(raw.getValue()));
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    //Raw Crosshairs:
    //If you are using raw targeting data, you can still utilize your calibrated crosshairs:
    
    public double[] getAdvanced_RawCrosshair(Advanced_Crosshair raw){
        double[] crosshars = new double[2];
        crosshars[0] = getAdvanced_RawCrosshair_X(raw);
        crosshars[1] = getAdvanced_RawCrosshair_Y(raw);
        return crosshars;
    }
    public double getAdvanced_RawCrosshair_X(Advanced_Crosshair raw) {
        NetworkTableEntry cxRaw = m_table.getEntry("cx" + Integer.toString(raw.getValue()));
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    public double getAdvanced_RawCrosshair_Y(Advanced_Crosshair raw) {
        NetworkTableEntry cyRaw = m_table.getEntry("cy" + Integer.toString(raw.getValue()));
        double y = cyRaw.getDouble(0.0);
        return y;
    }

}
