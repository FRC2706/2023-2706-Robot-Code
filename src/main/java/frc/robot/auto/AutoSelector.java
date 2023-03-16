package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AnalogSelectorSubsystem;

public class AutoSelector {
    
    int m_analogSelectorIndex = 0;
    private AnalogSelectorSubsystem analogSelector = null; 
 
     
    public AutoSelector()
    {
        InitFrcDashboard();

        analogSelector = AnalogSelectorSubsystem.getInstance();        
    }

    void InitFrcDashboard()
    {
        //use FRC Labview Dashboard
        String[] autoList = {
        "Id0_null", 
        "Id1_cube_0p5_top_charge_good",
        "Id2_cube_0p5_bottom_charge",
        "Id3_cube_0p5_middle_charge", 
        "Id4_cube_1p0_top",
        "Id5_cube_1p0_bottom",
        "Id6_cone_0p5_top_charge",
        "Id7_cone_0p5_middle1_charge",
        "Id8_cone_0p5_middle2_charge",
        "Id9_cone_0p5_bottom_charge",
        "Id10_cone_1p0_top",
        "Id11_cone_1p0_bottom",
        "Id12_BK_cube_2p0_bottom",
        "Id13_BK_cube_0p5_middle_charge",
        "Id14_BK_cone_0p5_charge",
        "Id15_place_pick_bottom2_charge_new",
        "Id16_place_pick_place_pick_place_bottom2",
        "Id17_place_pick_place_pick_place_bottom2_charge",
        "Id18_place_pick_place_pick_place_bottom_new",
        "Id19_cube_1p0_top_charge",
        "Id20_cone_2p0_bot"
        };
        SmartDashboard.putStringArray("Auto List", autoList );
    }

    public int getAutoId()
    {
        int autoId;
        String autoName = SmartDashboard.getString("Auto Selector", "Use Analog Selector");

        System.out.println("FRC Dashboard autoName: " +autoName);

        switch(autoName)
        {
            case "Select Autonomous ...  ":
                autoId = getAnalogSelectorIndex();
                break;
            case "Use Analog Selector":
                autoId = getAnalogSelectorIndex();
                break;
            case "Id0_null":
                autoId = 0;
                break;
            case "Id1_cube_0p5_top_charge_good":
                autoId = 1;
                break;
            case "Id2_cube_0p5_bottom_charge":
                autoId = 2;
                break;
            case "Id3_cube_0p5_middle_charge":
                autoId = 3;
                break;
            case "Id4_cube_1p0_top":
                autoId = 4;
                break;
            case "Id5_cube_1p0_bottom":
                autoId = 5;
                break;
            case "Id6_cone_0p5_top_charge":
                autoId = 6;
                break;
            case "Id7_cone_0p5_middle1_charge":
                autoId = 7;
                break;
            case "Id8_cone_0p5_middle2_charge":
                autoId = 8;
                break;
            case "Id9_cone_0p5_bottom_charge": 
                autoId = 9;
                break;
            case "Id10_cone_1p0_top": 
                autoId = 10; 
                break;
            case "Id11_cone_1p0_bottom": 
                autoId = 11;
                break;
            case "Id12_BK_cube_2p0_bottom": 
                autoId = 12;
                break;
            case "Id13_BK_cube_0p5_middle_charge": 
                autoId = 13;
                break;
            case "Id14_BK_cone_0p5_charge": 
                autoId = 14;
                break;
            case "Id15_place_pick_bottom2_charge_new":
                autoId = 15;
                break;
            case "Id16_place_pick_place_pick_place_bottom2":
                autoId = 16;
                break;
            case "Id17_place_pick_place_pick_place_bottom2_charge":
                autoId = 17;
                break;
            case "Id18_place_pick_place_pick_place_bottom_new":
                autoId = 18;
                break;
            case "Id19_cube_1p0_top_charge":
                autoId = 19;
                break;
            case "Id20_cone_2p0_bot":
                autoId = 20;
                break;
            default:
                autoId = getAnalogSelectorIndex();
                break;
 
        }
        return autoId; 
    }

    public int getAnalogSelectorIndex()
    {
        if (analogSelector != null){
            //Get value from selector
            m_analogSelectorIndex = analogSelector.getIndex();
            //If m_analogSelectorIndex is -1, set it to 0.
            if(m_analogSelectorIndex == -1)
            {
                m_analogSelectorIndex = 0;//do nothing
            }
        }
        else
        {
            m_analogSelectorIndex = 0;
        }

        return m_analogSelectorIndex;
    }
    
}
