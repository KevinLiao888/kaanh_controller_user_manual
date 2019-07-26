#include"forcecontrol.h"
#include <atomic>
#include <array>


using namespace aris::dynamic;
using namespace aris::plan;


namespace forcecontrol
{
	// 力控拖动——单关节或者6个轨迹相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈；电流控制 //
	struct MoveJRCParam
	{
		double kp_p, kp_v, ki_v;
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	static std::atomic_bool enable_moveJRC = true;
	auto MoveJRC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRCParam param;
		enable_moveJRC = true;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "kp_p")
			{
				param.kp_p = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "kp_v")
			{
				param.kp_v = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "ki_v")
			{
				param.ki_v = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		std::fill(target.mot_options.begin(), target.mot_options.end(),
			Plan::USE_TARGET_POS |
			Plan::USE_VEL_OFFSET);

	}
	auto MoveJRC::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRCParam&>(target.param);
		auto controller = target.controller;
        bool is_running{ true };
		static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double pqa[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		bool ds_is_all_finished{ true };
		bool md_is_all_finished{ true };

		//第一个周期，将目标电机的控制模式切换到电流控制模式
		if (target.count == 1)
		{

			is_running = true;
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_joint_pos_vec[i] = target.model->motionPool()[i].mp();
					controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
				}
			}
		}

		//最后一个周期将目标电机去使能
		if (!enable_moveJRC)
		{
			is_running = false;
		}
		if (!is_running)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					//controller->motionPool().at(i).setModeOfOperation(8);
					//controller->motionPool().at(i).setTargetPos(controller->motionAtAbs(i).actualPos());
					//target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
					auto ret = controller->motionPool().at(i).disable();
					if (ret)
					{
						ds_is_all_finished = false;
					}
				}
			}
		}

		//将目标电机由电流模式切换到位置模式
		if (!is_running&&ds_is_all_finished)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
                    auto &cm = controller->motionPool().at(i);
                    controller->motionPool().at(i).setModeOfOperation(8);
                    auto ret = cm.mode(8);
                    cm.setTargetPos(cm.actualPos());
					if (ret)
					{
						md_is_all_finished = false;
					}
				}
			}
		}

		//动力学
		for (int i = 0; i < 6; ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
			target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
			target.model->motionPool().at(i).setMa(0.0);
		}

		target.model->solverPool()[1].kinPos();
		target.model->solverPool()[1].kinVel();
		target.model->solverPool()[2].dynAccAndFce();

        double ft_offset = 0;
        static int counter=0;
		if (is_running)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
                    double p, v, pa, vt, va, voff, ft, foff;
					p = controller->motionAtAbs(i).actualPos();
					v = 0.0;
					pa = controller->motionAtAbs(i).actualPos();
					va = controller->motionAtAbs(i).actualVel();
					voff = v * 1000;
					foff = 0.0;
					vt = param.kp_p*(p - pa) + voff;
					//限制速度的范围在-1.0~1.0之间
					vt = std::max(-1.0, vt);
					vt = std::min(1.0, vt);

					vinteg[i] = vinteg[i] + vt - va;
					ft = param.kp_v*(vt - va) + param.ki_v*vinteg[i] + foff;
					//限制电流的范围在-400~400(千分数：额定电流是1000)之间
					ft = std::max(-400.0, ft);
					ft = std::min(400.0, ft);

					//拖动示教
                    auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
                    ft_offset = (f_vel_JRC[i] * controller->motionAtAbs(i).actualVel() + f_static_index_JRC[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

					ft_offset = std::max(-500.0, ft_offset);
					ft_offset = std::min(500.0, ft_offset);

                    controller->motionAtAbs(i).setTargetCur(ft_offset + target.model->motionPool()[i].mfDyn()*f2c_index[i]);

					//打印PID控制结果
					/*
					auto &cout = controller->mout();
					if (target.count % 100 == 0)
					{
						//cout << "ft:" << ft << "  " << "vt:" << vt << "  " << "va:" << va << "  " << "param.kp_v*(vt - va):" << param.kp_v*(vt - va) << "  " << "param.ki_v*vinteg[i]:" << param.ki_v*vinteg[i] << "    ";
						cout << "feedbackf:" << std::setw(10) << controller->motionAtAbs(i).actualCur()
							 << "f:" << std::setw(10) << ft_offset
							 << "p:" << std::setw(10) << p
							 << "pa:" << std::setw(10) << pa
							 << "va:" << std::setw(10) << va << std::endl;
					}
					*/
				}
			}
		}
		
		if (target.model->solverPool().at(1).kinPos())return -1;
		target.model->generalMotionPool().at(0).getMpq(pqa);

		// 打印电流 //
		auto &cout = controller->mout();
        if (target.count % 1000 == 0)
		{

			for (Size i = 0; i < param.joint_active_vec.size(); i++)
			{
				if (param.joint_active_vec[i])
				{
                    cout << "pos" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualPos() << ",";
                    cout << "vel" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualVel() << ",";
                    cout << "cur" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualCur() << ",";
				}
			}

			cout << "pq: ";
			for (Size i = 0; i < 7; i++)
			{
				cout << std::setw(6) << pqa[i] << " ";
			}

            cout << std::endl;
		}

		// log 位置、速度、电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < param.joint_active_vec.size(); i++)
		{
            //lout << std::setw(10) << controller->motionAtAbs(i).targetCur() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualPos() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualVel() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualCur() << " | ";

            lout << controller->motionAtAbs(i).actualPos() << " ";
            lout << controller->motionAtAbs(i).actualVel() << " ";
            lout << controller->motionAtAbs(i).actualCur() << " ";	
		}
		
		// log 末端pq值 //
		for (Size i = 0; i < 7; i++)
		{
			lout << pqa[i] << " ";
		}
		lout << std::endl;

		return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
	}
	auto MoveJRC::collectNrt(PlanTarget &target)->void {}
	MoveJRC::MoveJRC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJRC\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<Param name=\"kp_p\" default=\"1\"/>"
			"		<Param name=\"kp_v\" default=\"100\"/>"
			"		<Param name=\"ki_v\" default=\"0.1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 力控停止指令——停止MoveStop，去使能电机 //
	auto MoveStop::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			enable_moveJRC = false;
			target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

		}
	MoveStop::MoveStop(const std::string &name) :Plan(name)
		{
			command().loadXmlStr(
				"<Command name=\"moveStop\">"
				"</Command>");
		}
	
}
