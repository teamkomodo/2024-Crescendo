package frc.LimeLightVison;

import java.sql.Time;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.LimeLightVison.ControlMode;


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

    private bool isConnected(){
        resetPipelineLatency();
        Timer.delay(.5);
    }
}
