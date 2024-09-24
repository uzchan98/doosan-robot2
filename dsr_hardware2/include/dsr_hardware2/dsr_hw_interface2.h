/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor 
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "pinocchio/multibody/fwd.hpp"
#define _DEBUG_DSR_CTL      1

#define USE_FULL_LIB

#ifndef DSR_HARDWARE2__DR_HW_INTERFACE2_H
#define DSR_HARDWARE2__DR_HW_INTERFACE2_H

//ROS2 #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy

// #ifdef _OLD_ROS2_CONTROL_
//     #include <hardware_interface/joint_command_handle.hpp>
//     #include <hardware_interface/joint_state_handle.hpp>
// #else 
//     #include "hardware_interface/joint_handle.hpp"
// #endif

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// msg
#include "dsr_msgs2/msg/robot_error.hpp"
#include "dsr_msgs2/msg/robot_state.hpp"
#include "dsr_msgs2/msg/robot_stop.hpp"
#include "dsr_msgs2/msg/jog_multi_axis.hpp"

// moveit
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>  //ROS2 ???

#include "../../../common2/include/DRFLEx.h"
//TODO #include "../../../common2/include/dsr_serial.h"

#ifdef USE_FULL_LIB
#define _DEBUG_DSR_CTL      0

#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

//_____ defines for Dooan Robot Controller _______________
#define POINT_COUNT         6

// solution space
#define DR_SOL_MIN          0
#define DR_SOL_MAX          7

// posb seg_type
#define DR_LINE             0
#define DR_CIRCLE           1

// move reference
#define DR_BASE             0
#define DR_TOOL             1
#define DR_WORLD            2
#define DR_TC_USER_MIN      101
#define DR_TC_USER_MAX      200

// move mod
#define DR_MV_MOD_ABS       0
#define DR_MV_MOD_REL       1

// move reaction
#define DR_MV_RA_NONE       0
#define DR_MV_RA_DUPLICATE  0
#define DR_MV_RA_OVERRIDE   1

// move command type
#define DR_MV_COMMAND_NORM  0

// movesx velocity
#define DR_MVS_VEL_NONE     0
#define DR_MVS_VEL_CONST    1

// motion state
#define DR_STATE_IDLE       0
#define DR_STATE_INIT       1
#define DR_STATE_BUSY       2
#define DR_STATE_BLEND      3
#define DR_STATE_ACC        4
#define DR_STATE_CRZ        5
#define DR_STATE_DEC        6

// axis
#define DR_AXIS_X           0
#define DR_AXIS_Y           1
#define DR_AXIS_Z           2
#define DR_AXIS_A          10
#define DR_AXIS_B          11
#define DR_AXIS_C          12

// collision sensitivity
#define DR_COLSENS_DEFAULT 20
#define DR_COLSENS_MIN      1   
#define DR_COLSENS_MAX    300

// speed
#define DR_OP_SPEED_MIN     1
#define DR_OP_SPEED_MAX   100

// stop
#define DR_QSTOP_STO        0
#define DR_QSTOP            1
#define DR_SSTOP            2
#define DR_HOLD             3

#define DR_STOP_FIRST       DR_QSTOP_STO
#define DR_STOP_LAST        DR_HOLD

// condition
#define DR_COND_NONE        -10000

// digital I/O
#define DR_DIO_MIN_INDEX    1
#define DR_DIO_MAX_INDEX    16  

// tool digital I/O
#define DR_TDIO_MIN_INDEX   1
#define DR_TDIO_MAX_INDEX   6

// I/O value
#define ON                  1
#define OFF                 0

// Analog I/O mode
#define DR_ANALOG_CURRENT   0
#define DR_ANALOG_VOLTAGE   1

// modbus type
#define DR_MODBUS_DIG_INPUT     0
#define DR_MODBUS_DIG_OUTPUT    1
#define DR_MODBUS_REG_INPUT     2
#define DR_MODBUS_REG_OUTPUT    3
#define DR_DISCRETE_INPUT       0
#define DR_COIL                 1
#define DR_INPUT_REGISTER       2
#define DR_HOLDING_REGISTER     3

#define DR_MODBUS_ACCESS_MAX    32
#define DR_MAX_MODBUS_NAME_SIZE 32

// tp_popup pm_type
#define DR_PM_MESSAGE           0
#define DR_PM_WARNING           1
#define DR_PM_ALARM             2

// tp_get_user_input type
#define DR_VAR_INT              0
#define DR_VAR_FLOAT            1
#define DR_VAR_STR              2
#define DR_VAR_BOOL             3   

// len
#define DR_VELJ_DT_LEN          6
#define DR_ACCJ_DT_LEN          6

#define DR_VELX_DT_LEN          2
#define DR_ACCX_DT_LEN          2

#define DR_ANGLE_DT_LEN         2
#define DR_COG_DT_LEN           3
#define DR_WEIGHT_DT_LEN        3
#define DR_VECTOR_DT_LEN        3
#define DR_ST_DT_LEN            6
#define DR_FD_DT_LEN            6
#define DR_DIR_DT_LEN           6
#define DR_INERTIA_DT_LEN       6
#define DR_VECTOR_U1_LEN        3
#define DR_VECTOR_V1_LEN        3

#define DR_AVOID                0
#define DR_TASK_STOP            1

#define DR_FIFO                 0
#define DR_LIFO                 1

#define DR_FC_MOD_ABS           0
#define DR_FC_MOD_REL           1

#define DR_GLOBAL_VAR_TYPE_BOOL         0
#define DR_GLOBAL_VAR_TYPE_INT          1
#define DR_GLOBAL_VAR_TYPE_FLOAT        2
#define DR_GLOBAL_VAR_TYPE_STR          3
#define DR_GLOBAL_VAR_TYPE_POSJ         4
#define DR_GLOBAL_VAR_TYPE_POSX         5
#define DR_GLOBAL_VAR_TYPE_UNKNOWN      6

#define DR_IE_SLAVE_GPR_ADDR_START      0
#define DR_IE_SLAVE_GPR_ADDR_END       23
#define DR_IE_SLAVE_GPR_ADDR_END_BIT   63

#define DR_DPOS                         0
#define DR_DVEL                         1

#define DR_HOME_TARGET_MECHANIC         0
#define DR_HOME_TARGET_USER             1

#define DR_MV_ORI_TEACH                 0    
#define DR_MV_ORI_FIXED                 1    
#define DR_MV_ORI_RADIAL                2    

#define DR_MV_APP_NONE                  0
#define DR_MV_APP_WELD                  1
//________________________________________________________

#endif 
typedef struct {
    int	    nLevel;         // INFO =1, WARN =2, ERROR =3 
    int	    nGroup;         // SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int	    nCode;          // error code 
    char    strMsg1[MAX_STRING_SIZE];   // error msg 1
    char    strMsg2[MAX_STRING_SIZE];   // error msg 2
    char    strMsg3[MAX_STRING_SIZE];   // error msg 3
} DR_ERROR, *LPDR_ERROR;

typedef struct {
    int     nRobotState;
    char    strRobotState[MAX_SYMBOL_SIZE];
    float   fCurrentPosj[NUM_JOINT];
    float   fCurrentPosx[NUM_TASK];
    float   fCurrentToolPosx[NUM_TASK];

    int     nActualMode;
    int     nActualSpace;
    
    float   fJointAbs[NUM_JOINT];
    float   fJointErr[NUM_JOINT];
    float   fTargetPosj[NUM_JOINT];
    float   fTargetVelj[NUM_JOINT];
    float   fCurrentVelj[NUM_JOINT];

    float   fTaskErr[NUM_TASK];
    float   fTargetPosx[NUM_TASK];
    float   fTargetVelx[NUM_TASK];
    float   fCurrentVelx[NUM_TASK];
    int     nSolutionSpace;
    float   fRotationMatrix[3][3];

    float   fDynamicTor[NUM_JOINT];
    float   fActualJTS[NUM_JOINT];
    float   fActualEJT[NUM_JOINT];
    float   fActualETT[NUM_JOINT];

    double  dSyncTime;
    int     nActualBK[NUM_JOINT];
    int     nActualBT[NUM_BUTTON];
    float   fActualMC[NUM_JOINT];
    float   fActualMT[NUM_JOINT];
    bool    bCtrlBoxDigitalOutput[16];
    bool    bCtrlBoxDigitalInput[16];
    bool    bFlangeDigitalOutput[6];
    bool    bFlangeDigitalInput[6];

    int     nRegCount;
    string  strModbusSymbol[100];
    int     nModbusValue[100];
  
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;

    //--- The following variables have been updated since version M2.50 or higher. ---
	//ROBOT_MONITORING_WORLD
	float   fActualW2B[6];
	float   fCurrentPosW[2][6];
	float   fCurrentVelW[6];
	float   fWorldETT[6];
	float   fTargetPosW[6];
	float   fTargetVelW[6];
	float   fRotationMatrixWorld[3][3];

	//ROBOT_MONITORING_USER
	int     iActualUCN;
	int     iParent;
	float   fCurrentPosU[2][6];
	float   fCurrentVelU[6];
	float   fUserETT[6];
	float   fTargetPosU[6];
	float   fTargetVelU[6];
	float   fRotationMatrixUser[3][3];

    //READ_CTRLIO_INPUT_EX
	float   fActualAI[6];
	bool    bActualSW[3];
	bool    bActualSI[2];
	int     iActualAT[2];

	//READ_CTRLIO_OUTPUT_EX
	float   fTargetAO[2];
	int     iTargetAT[2];

	//READ_ENCODER_INPUT
	bool    bActualES[2];
	int     iActualED[2];
	bool    bActualER[2];
    //---------------------------------------------------------------------------------

} DR_STATE, *LPDR_STATE;
std::string m_name;
std::string m_host;
std::string m_mode;
std::string m_model;
std::string m_gripper;
std::string m_mobile;
unsigned int m_rate;
unsigned int m_standby;
bool m_command;
unsigned int m_port;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub_;
typedef struct _ROBOT_JOINT_DATA
{
    double cmd;
    double pos;
    double vel;
    double eff;
} ROBOT_JOINT_DATA, *LPROBOT_JOINT_DATA;
using namespace DRAFramework;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace dsr_hardware2{

    class HARDWARE_INTERFACE_PUBLIC DRHWInterface : public hardware_interface::SystemInterface
    {
    public:
        std::vector<std::string> joint_names = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"
            };
        int m_nVersionDRCF;
        bool init_check;
        bool m_bCommand_;
        std::array<float, NUM_JOINT> m_fCmd_;
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces);
        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
        ~DRHWInterface();

    protected:
        /// The size of this vector is (standard_interfaces_.size() x nr_joints)
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_efforts_command_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_efforts_;
        std::vector<double> ft_states_;
        std::vector<double> ft_command_;

        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
            {"position", {}}, {"velocity", {}}, {"effort", {}}};
        };

        enum ControlMode {
            UNKNOWN = 0,
            POSITION, 
            VELOCITY,
            TORQUE
        } control_mode_;

        pinocchio::Model robot_mdl_;
        pinocchio::Data robot_data_;


}
#ifdef USE_FULL_LIB
    class DSRInterface : public rclcpp::Node
    {
    public:
        DSRInterface();
        virtual ~DSRInterface();

        int MsgPublisher_RobotState();
        int MsgPublisher_JointState();

        ///int MsgPublisher_RobotError();  현재 미사용 : DRHWInterface::OnLogAlarm 에서 바로 퍼블리싱 함.
        static void OnHommingCompletedCB();
        static void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause);
        static void OnMonitoringDataCB(const LPMONITORING_DATA pData);
        static void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
        static void OnTpInitializingCompletedCB();

        static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
        static void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
        static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
        static void OnMonitoringStateCB(const ROBOT_STATE eState);
        static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
        static void OnLogAlarm(LPLOG_ALARM pLogAlarm);
        void DSRInterfaceNode();

        std::string GetRobotName();
        std::string GetRobotModel();

    private:
        
        //ROS2 bool m_bIsEmulatorMode; -> g_bIsEmulatorMode
        
        // rclcpp::Node::SharedPtr private_nh_;	

        std::string m_strRobotName;
        std::string m_strRobotModel;
        std::string m_strRobotGripper;


        //----- Publisher -------------------------------------------------------------
        rclcpp::Publisher<dsr_msgs2::msg::RobotState>::SharedPtr            m_PubRobotState;
        ///rclcpp::Publisher<dsr_msgs2::msg::RobotError>::SharedPtr            m_PubRobotError;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      m_PubtoGazebo;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 m_PubSerialWrite;
        ///TODO rclcpp::Publisher<dsr_msgs::msg::JogMultiAxis>::SharedPtr  m_PubJogMultiAxis;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_PubJointState;  //add for TEST
        
        //----- Subscriber ------------------------------------------------------------
        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_sub_joint_trajectory;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_sub_test;
        


        void trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) const;

        void testSubCallback(const std_msgs::msg::String::SharedPtr msg) const;

        //----- Threads ------------------------------------------------------------------
        boost::thread m_th_subscribe;   //subscribe thread
        boost::thread m_th_publisher;   //publisher thread
        boost::thread m_th_publisher_joint_states;   //publisher thread

        static void thread_subscribe(rclcpp::Node::SharedPtr nh);
        static void thread_publisher(DSRInterface* pDSRInterface, rclcpp::Node::SharedPtr nh, int nPubRate);
        static void thread_publisher_direct_access_joint_states(DSRInterface* pDSRInterface, rclcpp::Node::SharedPtr nh, int nPubRate);// '/joint_state'에 직접 퍼블리싱하는 함수(현재 미사용) 
                                                                                                                                         // ros2_control를 통해서 퍼블리싱 함. 

        DR_STATE m_stDrState;
        DR_ERROR m_stDrError;
    
    };
#endif

#endif // end
