#include<iostream>
//程序引用sris库的头文件
#include<aris.hpp>

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
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

// 创建轨迹
struct MoveEAPParam
{
	std::vector<bool> axis_active_vec;
    std::vector<aris::Size> total_count_vec;
	std::vector<double> axis_begin_pos_vec;
	std::vector<double> axis_pos_vec;
	std::vector<double> axis_vel_vec;
	std::vector<double> axis_acc_vec;
	std::vector<double> axis_dec_vec;
	bool abs;
};
class MoveEAP : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        auto c = target.controller;
		MoveEAPParam param;
        param.axis_begin_pos_vec.resize(c->motionPool().size());

		aris::core::Calculator cal;

		for (auto &p : params)
		{
			if (p.first == "all")
			{
				param.axis_active_vec.resize(c->motionPool().size(), true);
			}
			else if (p.first == "motion_id")
			{
				param.axis_active_vec.resize(c->motionPool().size(), false);
				param.axis_active_vec.at(std::stoi(p.second)) = true;
			}
			else if (p.first == "pos")
			{
                aris::core::Matrix mat = cal.calculateExpression(p.second);
				if (mat.size() == 1)param.axis_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else
				{
					param.axis_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.axis_pos_vec.begin());
				}
			}
			else if (p.first == "vel")
			{	
                auto v = cal.calculateExpression(p.second);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
				}
				else if (v.size() == c->motionPool().size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}
				//对速度值超限进行保护
                for (aris::Size i = 0; i < param.axis_vel_vec.size(); ++i)
				{
					if (param.axis_vel_vec[i] > 0.2)
					{
						param.axis_vel_vec[i] = 0.2;
					}
					if (param.axis_vel_vec[i] < -0.2)
					{
						param.axis_vel_vec[i] = -0.2;
					}
				}
			}
			else if (p.first == "acc")
			{
                auto a = cal.calculateExpression(p.second);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
				}
				else if (a.size() == c->motionPool().size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}
				//对加速度值超限进行保护
                for (aris::Size i = 0; i < param.axis_acc_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 2.0)
					{
						param.axis_acc_vec[i] = 2.0;
					}
					if (param.axis_acc_vec[i] < -2.0)
					{
						param.axis_acc_vec[i] = -2.0;
					}
				}
			}
			else if (p.first == "dec")
			{
                auto d = cal.calculateExpression(p.second);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
				}
				else if (d.size() == c->motionPool().size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}
				//对减速度值超限进行保护
                for (aris::Size i = 0; i < param.axis_dec_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 2.0)
					{
						param.axis_dec_vec[i] = 2.0;
					}
					if (param.axis_dec_vec[i] < -2.0)
					{
						param.axis_dec_vec[i] = -2.0;
					}
				}
			}
			else if (p.first == "abs")
			{
				param.abs = std::stod(p.second);
			}
		}

		target.param = param;

		target.option =
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveEAPParam&>(target.param);
		// 访问主站 //
        auto controller = target.controller;
		// 第一个count，取各个电机的当前位置
		if (target.count == 1)
		{
			for (int i = 0; i < param.axis_active_vec.size(); i++)
			{
				param.axis_begin_pos_vec[i] = controller->motionAtAbs(i).targetPos();
			}
		}
        aris::Size total_count{ 1 };
		double p, v, a;
        aris::Size t_count;
		//绝对轨迹//
		if (param.abs)
		{
			for (int i = 0; i < param.axis_active_vec.size(); i++)
			{
				//梯形轨迹规划函数moveAbsolute对输入的量(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000)都会取绝对值
				//然后规划出当前count的指令位置p，指令速度v，指令加速度a，以及本梯形轨迹的总count数t_count
				if(param.axis_active_vec[i])
				{ 
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, t_count);
					controller->motionAtAbs(i).setTargetPos(p);
					total_count = std::max(total_count, t_count);
				}
			}
		}
		//相对轨迹//
		else
		{
			for (int i = 0; i < param.axis_active_vec.size(); i++)
			{
				if (param.axis_active_vec[i])
				{
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, t_count);
					controller->motionAtAbs(i).setTargetPos(p);
					total_count = std::max(total_count, t_count);
				}
			}	
		}

		// 每1000ms打印 目标位置、实际位置、实际速度、实际电流 //
		auto &cout = controller->mout();
		if (target.count % 1000 == 0)
		{
			for (int i = 0; i < param.axis_active_vec.size(); i++)
			{
				cout << i <<":  "<< controller->motionAtAbs(i).targetPos() << "  " << controller->motionAtAbs(i).actualPos() << "  " << controller->motionAtAbs(i).actualVel() << "  " << controller->motionAtAbs(i).actualCur() << std::endl;
			}
		}
		// log 目标位置、实际位置、实际速度、实际电流 //
		
		auto &lout = controller->lout();
		for (int i = 0; i < param.axis_active_vec.size(); i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << "  " << controller->motionAtAbs(i).actualPos() << "  " << controller->motionAtAbs(i).actualVel() << "  " << controller->motionAtAbs(i).actualCur() << "  ";
		}
		lout << std::endl;
		// 返回total_count - target.count给aris实时核，值为-1，报错；值为0，结束；值大于0，继续执行下一个count
		return total_count - target.count;
	}

	auto virtual collectNrt(PlanTarget &target)->void {}

    explicit MoveEAP(const std::string &name = "MoveEAP_plan") :Plan(name)
	{
		command().loadXmlStr(
			"<moveEAP default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all/>"
			"			<motion_id default=\"0\"/>"
			"		</unique>"
			"		<pos default=\"0.02\"/>"
			"		<vel default=\"0.1\"/>"
			"		<acc default=\"0.5\"/>"
			"		<dec default=\"0.5\"/>"
			"		<abs default=\"1\"/>"
			"	</group>"
			"</moveEAP>");
	}
};

// 将轨迹MoveEAP添加到轨迹规划池planPool中
auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Recover>();
    plan_root->planPool().add<aris::plan::Sleep>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
	rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.0,0.0}\" abbreviation=\"p\"/>");

    plan_root->planPool().add<aris::plan::MoveL>();
	plan_root->planPool().add<aris::plan::MoveJ>();
	plan_root->planPool().add<MoveEAP>();
	plan_root->planPool().add<aris::plan::Show>();

	return plan_root;
}

// 主函数
int main(int argc, char *argv[])
{
	//创建Ethercat主站对象
    aris::control::EthercatMaster mst;
	//自动扫描，连接从站
    mst.scan();
    std::cout<<mst.xmlString()<<std::endl;

	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());

    std::cout<<"start thread"<<std::endl;
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
			LOG_ERROR << e.what() << std::endl;
		}
	}
}
