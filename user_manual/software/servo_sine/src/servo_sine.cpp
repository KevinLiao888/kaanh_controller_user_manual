#include<iostream>
//程序引用sris库的头文件
#include<aris.hpp>

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;
/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针
    std::unique_ptr*/
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	
{
	/*创建std::unique_ptr实例*/
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	/*定义Ethercat控制器的xmal文件
		phy_id指电机的序号，由两个电机，此处先完成0号电机的xml配置
		product_code、vendor_id等数据由控制器读取
		min_pos、max_pos与电缸 的行程有关，前者小于电缸的最小位置0mm，后
		     者大于电缸的最大形成100mm
		max_vel根据电缸的额定转速和行程来计算，即
		     3000（转速）*4（导程）/60/1000=0.2（单位m/s）
		min_vel与max_vel大小相等，方向相反
		max_acc按照经验定义为速度的10倍
		max_pos_following_error、max_vel_following_error由经验数据确定
		home_pos指电缸的初始位置，定义为0
		pos_factor指电缸在推进1米的情况下，控制器的电信号个数，通过查询电机
		     为23bit，则电机转动一圈的情况下电脉冲的次数是2^23=8388608个，电缸推
		     进一米需要转动250圈，则pos_factor=8388608*250=2097152000
		pos_offset是指电机在断电重启后电机的初始位置距离0点的偏差*/
	std::string xml_str =
        "<m_servo_press type=\"EthercatMotion\" phy_id=\"0\" product_code=\"0x60380005\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-0.1\" max_pos=\"0.15\" max_vel=\"0.2\" min_vel=\"-0.2\""
		" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\"2097152000\" pos_offset=\"0.00383346\">"
		"	<sm_pool type=\"SyncManagerPoolObject\">"
		"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
		"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
		"		<sm type=\"SyncManager\" is_tx=\"false\">"
		"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
		"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
		"				<offset_vel index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<offset_tor index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
		"			</index_1600>"
		"		</sm>"
		"		<sm type=\"SyncManager\" is_tx=\"true\">"
		"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
		"				<status_word index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"				<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"				<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"				<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
		"				<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
		"			</index_1a00>"
		"		</sm>"
		"	</sm_pool>"
		"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
		"	</sdo_pool>"
		"</m_servo_press>";
	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
	//以下phy_i赋值为1，完成1号电机的xml配置,参数的配置同上
	std::string xml_str1 =
        "<m_servo_press type=\"EthercatMotion\" phy_id=\"1\" product_code=\"0x60380005\""
        " vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-0.1\" max_pos=\"0.15\" max_vel=\"0.2\" min_vel=\"-0.2\""
        " max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\"2097152000\" pos_offset=\"0.0032925\">"
		"	<sm_pool type=\"SyncManagerPoolObject\">"
		"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
		"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
		"		<sm type=\"SyncManager\" is_tx=\"false\">"
		"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
		"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
		"				<offset_vel index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<offset_tor index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
		"			</index_1600>"
		"		</sm>"
		"		<sm type=\"SyncManager\" is_tx=\"true\">"
		"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
		"				<status_word index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"				<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"				<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"				<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
		"				<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
		"			</index_1a00>"
		"		</sm>"
		"	</sm_pool>"
		"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
		"	</sdo_pool>"
		"</m_servo_press>";
	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str1);

	return controller;
};

//moveJS中的s是指sin函数
//定义结构体，添加参数，其中有总时间total_time，步长step_size
struct MoveJSParam
{
	int total_time;
	int left_time;
    double step_size;//
};

//定义类，有几个类，就有几个控制算法
class MoveJS : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        MoveJSParam p;
		//将total_time的数值转化为int类型并将值赋给p中的参数total_time
		p.total_time = std::stoi(params.at("total_time"));
		//将step_size的数值转化为double类型并将值赋给p中的参数step_size
        p.step_size=std::stod(params.at("step_size"));//
		p.left_time = 0;
		target.param = p;
        target.option =
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
                aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
                aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
                aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
        
        auto controller = target.controller;
        auto &p = std::any_cast<MoveJSParam&>(target.param);

        //p.total_time
		//添加电缸的起始位置参数beginpos[2]
        static double beginpos[2];
		//当发起第一个脉冲时检查电机的初始位置，并赋值给beginpos[2]
        if(target.count == 1)
        {
			//打印两个电缸的初始位置beginpos[2]
            controller->mout()<<"mot1:" << controller->motionPool()[0].actualPos()<<std::endl;
            controller->mout()<<"mot2:" << controller->motionPool()[1].actualPos()<<std::endl;
			//获取两个电缸的初始位置，并赋值给参数
            beginpos[0] = controller->motionPool()[0].actualPos();
            beginpos[1] = controller->motionPool()[1].actualPos();
        }
		/*定义两个电缸的sine轨迹曲线控制函数,控制时间为一个函数周期
		beginpos[0]后面的减号是对应着电缸推进到最远的位置*/
        controller->motionPool()[0].setTargetPos(beginpos[0] - p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI))) ; 
        controller->motionPool()[1].setTargetPos(beginpos[1] - p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI))) ;

        return p.total_time - target.count;
	}
	auto virtual collectNrt(PlanTarget &target)->void {}
    explicit MoveJS(const std::string &name = "myplan"):Plan(name)
	{
		/*导入规划函数的myplan的xml文件
			将总时间total_time定义为5000，由于实时核1ms控制一次，所以总时间为5s
			将步长step_size赋值为0.045m，，则电缸推动的最远距离为2*0.045=0.09
			在电缸的行程0.1之内*/
		command().loadXmlStr(
			"<myplan>"
            "	<group type=\"GroupParam\">"
            "	    <total_time type=\"Param\" default=\"5000\"/>" //
            "       <step_size type=\"Param\" default=\"0.045\"/>"
            "   </group>"
            "</myplan>");
	}
};

auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Recover>();
    plan_root->planPool().add<aris::plan::Sleep>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
	rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5,0.01}\" abbreviation=\"p\"/>");

    plan_root->planPool().add<aris::plan::MoveL>();
	plan_root->planPool().add<aris::plan::MoveJ>();
	plan_root->planPool().add<MoveJS>();
	plan_root->planPool().add<aris::plan::Show>();

	
	return plan_root;
}

int main(int argc, char *argv[])
{
	//创建Ethercat主站对象
    aris::control::EthercatMaster mst;
	//自动扫描，连接从站
    //mst.scan();
    std::cout<<mst.xmlString()<<std::endl;
	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());

    std::cout<<"start"<<std::endl;
	//启动线程
	cs.start();
	//getline是将输入的值赋值给command_in
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto id = cs.executeCmd(aris::core::Msg(command_in));
			std::cout << "command id:" << id << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			//LOG_ERROR << e.what() << std::endl;
		}
	}

	char ch;
	std::cin >> ch;
}
