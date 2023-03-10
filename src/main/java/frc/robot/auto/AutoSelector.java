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
        "Id1_leave_top",
        "Id2_leave_middle_around_",
        "Id3_leave_middle_through_", 
        "Id4_leave_bottom",
        "Id5_leave_balance_top",
        "Id6_leave_balance_middle_around_",
        "Id7_leave_balance_middle_through_",
        "Id8_leave_balance_bottom",
        "Id9_place_pick_top",
        "Id10_place_pick_middle_around_",
        "Id11_place_pick_middle_through_",
        "Id12_place_pick_bottom",
        "Id13_place_pick_balance_top",
        "Id14_place_pick_balance_middle",
        "Id15_place_pick_balance_bottom",
        "Id16_place_pick_place_balance_top",
        "Id17_place_pick_place_balance_middle",
        "Id18_place_pick_place_balance_bottom",
        "Id19_place_pick_place_top",
        "Id20_place_pick_place_middle",
        "Id21_place_pick_place_bottom",
        "Id22_place_pick_place_pick_place_top",
        "Id23_place_pick_place_pick_place_middle",
        "Id24_place_pick_place_pick_place_bottom",
        "Id25_Practice1",
        "Id26_Practice2",
        "Id27_place_pick_place_pick_place_bottom_new",
        "Id28_place_pick_bottom_charge_new",
        "Id29_place_pick_top_charge_new",
        "Id30_place_pick_place_pick_place_top2",
        };
        SmartDashboard.putStringArray("Auto List", autoList );
    }

    public int getAutoId()
    {
        int autoId;
        String autoName = SmartDashboard.getString("Auto Selector", "Use Analog Selector");

        System.out.println(autoName);

        switch(autoName)
        {
            case "Select Autonomous...  ":
                autoId = getAnalogSelectorIndex();
                break;
            case "Use Analog Selector":
                autoId = getAnalogSelectorIndex();
                break;
            case "Id0_null":
                autoId = 0;
                break;
            case "Id1_leave_top":
                autoId = 1;
                break;
            case "Id2_leave_middle_around_":
                autoId = 2;
                break;
            case "Id3_leave_middle_through":
                autoId = 3;
                break;
            case "Id4_leave_bottom":
                autoId = 4;
                break;
            case "Id5_leave_balance_top":
                autoId = 5;
                break;
            case "Id6_leave_balance_middle_around_":
                autoId = 6;
                break;
            case "Id7_leave_balance_middle_through_":
                autoId = 7;
                break;
            case "Id8_leave_balance_bottom":
                autoId = 8;
                break;
            case "Id9_place_pick_top": //1.0_top
                autoId = 9;
                break;
            case "Id10_place_pick_middle_around_": //1.0_middle_around
                autoId = 10; 
                break;
            case "Id11_place_pick_middle_through_": //1.0_middle_through
                autoId = 11;
                break;
            case "Id12_place_pick_bottom": //1.0_bottom
                autoId = 12;
                break;
            case "Id13_place_pick_balance_top": //1.0_balance_top
                autoId = 13;
                break;
            case "Id14_place_pick_balance_middle": //1.0_balance_middle
                autoId = 14;
                break;
            case "Id15_place_pick_balance_bottom": //1.0_balance_bottom
                autoId = 15;
                break;
            case "Id16_place_pick_place_balance_top": //1.5_balance_top
                autoId = 16;
                break;
            case "Id17_place_pick_place_balance_middle": //1.5_balance_middle
                autoId = 17;
                break;
            case "Id18_place_pick_place_balance_bottom": //1.5_balance_bottom
                autoId = 18;
                break;
            case "Id19_place_pick_place_top": //1.5_top
                autoId = 19;
                break;
            case "Id20_place_pick_place_middle": //1.5_middle
                autoId = 20;
                break;
            case "Id21_place_pick_place_bottom": //1.5_bottom
                autoId = 21;
                break;
            case "Id22_place_pick_place_pick_place_top": //2.5_top
                autoId = 22;
                break;
            case "Id23_place_pick_place_pick_place_middle": //2.5_middle
                autoId = 23;
                break;
            case "Id24_place_pick_place_pick_place_bottom": //2.5_bottom
                autoId = 24;
                break;
            case "Id25_Practice1":
                autoId = 25;
                break;
            case "Id26_Practice2":
                autoId = 26;
                break;
            case "Id27_place_pick_place_pick_place_bottom_new": //2.5_bottom_new
                autoId = 27;
                break;
            case "Id28_place_pick_bottom_charge_new": //1.0_bottom_charge_new
                autoId = 28;
                break;
            
            case "Id29_place_pick_top_charge_new": //1.0_top_charge_new
                autoId = 29;
                break;
            
            case "Id30_place_pick_place_pick_place_top2": //2.5_top1
                autoId = 30;
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
