#include <algorithm>
#include"kaanh.h"
#include"forcecontrol.h"


using namespace aris::dynamic;
using namespace aris::plan;


namespace kaanh
{
	auto createControllerSanXiang()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

        for (aris::Size i = 0; i < 6; ++i)
		{

#ifdef WIN32
            double pos_offset[6]
			{
                0,   0,	  0,   0,   0,   0
			};
#endif
#ifdef UNIX
            double pos_offset[6]
            {
               0.85575155570192 , 2.40999766789812 , -2.37962964198803  ,-1.55737399489649  ,-1.62357485327349,  1.8732901476257
            };//0.856,     -2.988,    -2.795,     -1.558 ,    1.526,   1.873
#endif
            double pos_factor[6]
			{
                1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 524288.0 / 2 / PI, 524288.0 / 2 / PI
            };
            double max_pos[6]
			{
                180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	270.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 210.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
			};
            double min_pos[6]
			{
                -180.0 / 360 * 2 * PI, -60.0 / 360 * 2 * PI, -30.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -30.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI
			};
            double max_vel[6]
			{
                0.4, 0.4, 0.4, 0.4, 0.6, 0.6
			};
            double max_acc[6]
			{
//                3.3, 3.3*2, 3.3, 3.3, 7.5, 7.5
//                10, 10, 10, 10, 25, 25
                  20,20,20,20,50,50
			};

			std::string xml_str =
				"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x201\""
				" vendor_id=\"0x000022D2\" revision_num=\"0x0a000002\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotion>";
			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}


#ifdef UNIX
		dynamic_cast<aris::control::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();

		dynamic_cast<aris::control::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();
#endif

		std::cout << controller->xmlString() << std::endl;

		return controller;
	};
	auto createModelSanXiang()->std::unique_ptr<aris::dynamic::Model>
	{
        aris::dynamic::PumaParam param;
        param.d1 = 0.2511;
        param.a1 = 0.0;
        param.a2 = 0.375;
        param.d3 = 0.0;
        param.a3 = 0.0;
        param.d4 = 0.33;

        param.tool0_pe[2] = 0.2008;

        auto model = aris::dynamic::createModelPuma(param);
        /*
        //根据tool0，添加一个tool1，tool1相对于tool0在x方向加上0.1m//
        auto &tool0 = model->partPool().back().markerPool().findByName("general_motion_0_i");//获取tool0

        double pq_ee_i[7];
        s_pm2pq(*tool0->prtPm(), pq_ee_i);
        pq_ee_i[0] += 0.1;//在tool0的x方向加上0.1m

        double pm_ee_i[16];
        s_pq2pm(pq_ee_i, pm_ee_i);

        auto &tool1 = model->partPool().back().markerPool().add<Marker>("tool1", pm_ee_i);//添加tool1

        //在根据tool1位姿反解到每一个关节时，需要调用下面两行代码来实现
        //tool1.setPm(pm_ee_i);
        //model->generalMotionPool()[0].updMpm();
        */
        return std::move(model);
	}

    auto HomeFix::executeRT(PlanTarget &target)->int
    {
        for(auto &check_op:target.mot_options)check_op = aris::plan::Plan::NOT_CHECK_ENABLE;

        auto controller = target.controller;

        auto &cout = controller->mout();
        cout <<"before fix:"<<std::endl;
       for(int i=0;i<6;++i)
       {
           auto &mot = static_cast<aris::control::EthercatMotion&>(controller->motionPool()[i]);
            cout <<mot.actualPos()<<"  ";
       }
       cout <<std::endl;


       for(int i=0;i<6;++i)
       {
           auto &mot = static_cast<aris::control::EthercatMotion&>(controller->motionPool()[i]);

           if(mot.actualPos()>aris::PI) mot.setPosOffset(2*aris::PI + mot.posOffset());
           if(mot.actualPos()<-aris::PI) mot.setPosOffset(-2*aris::PI + mot.posOffset());
       }

       cout <<"after fix:"<<std::endl;
      for(int i=0;i<6;++i)
      {
          auto &mot = static_cast<aris::control::EthercatMotion&>(controller->motionPool()[i]);
          cout <<mot.actualPos()<<"  ";
      }
      cout <<std::endl;

        return 0;
    }
    HomeFix::HomeFix(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"hmf\">"
            "</Command>");
    }


	struct ShowAllParam
	{
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_pq_vec;
	};
	auto ShowAll::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ShowAllParam param;
		param.axis_pos_vec.clear();
		param.axis_pq_vec.clear();
		param.axis_pos_vec.resize(6, 0.0);
		param.axis_pq_vec.resize(7, 0.0);
		target.param = param;

		std::fill(target.mot_options.begin(), target.mot_options.end(),
			Plan::USE_TARGET_POS);
	}
	auto ShowAll::executeRT(PlanTarget &target)->int
	{
		// 访问主站 //
		auto controller = target.controller;
		auto &param = std::any_cast<ShowAllParam&>(target.param);

		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
			{
#ifdef UNIX
                //target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
				param.axis_pos_vec[i] = controller->motionAtAbs(i).actualPos();
#endif

#ifdef WIN32
				param.axis_pos_vec[i] = target.model->motionPool().at(i).mp();
#endif
			}
			target.model->generalMotionPool().at(0).getMpq(param.axis_pq_vec.data());
		}




		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		cout << "current pq:" << std::endl;
		for (Size i = 0; i < 7; i++)
		{
			cout << param.axis_pq_vec[i] << " ";
		}
		cout << std::endl;
		cout << "current pos:" << std::endl;
		for (Size i = 0; i < 6; i++)
		{
			cout << param.axis_pos_vec[i] << "  ";
		}
		cout << std::endl;

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << param.axis_pos_vec[i] << " ";
		}
		lout << std::endl;
		return 0;
	}
	auto ShowAll::collectNrt(PlanTarget &target)->void {}
	ShowAll::ShowAll(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"sha\">"
			"</Command>");
	}


	struct MoveJRParam
	{
		double begin_pos, target_pos, vel, acc, dec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second);
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
		}

		target.param = param;

        std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);
	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;
		

		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
                    param.begin_pos = target.controller->motionAtAbs(i).actualPos();
				}
			}
		}
		
		//梯形轨迹//
		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
                //target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}

		//3D模型同步
        //if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "target_pos" << ":" << param.target_pos << " ";
			cout << "vel" << ":" << param.vel << " ";
			cout << "acc" << ":" << param.acc << " ";
			cout << "dec"  << ":" << param.dec << " ";
			
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
					cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
				}
			}

			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		<Param name=\"pos\" default=\"0.2\"/>"
            "		<Param name=\"vel\" abbreviation=\"v\" default=\"0.05\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<UniqueParam default=\"check_none\">"
			"			<Param name=\"check_all\"/>"
			"			<Param name=\"check_none\"/>"
			"			<GroupParam>"
			"				<UniqueParam default=\"check_pos\">"
			"					<Param name=\"check_pos\"/>"
			"					<Param name=\"not_check_pos\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_pos_max\">"
			"							<Param name=\"check_pos_max\"/>"
			"							<Param name=\"not_check_pos_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_min\">"
			"							<Param name=\"check_pos_min\"/>"
			"							<Param name=\"not_check_pos_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous\">"
			"							<Param name=\"check_pos_continuous\"/>"
			"							<Param name=\"not_check_pos_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_at_start\">"
			"							<Param name=\"check_pos_continuous_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order\">"
			"							<Param name=\"check_pos_continuous_second_order\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
			"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_following_error\">"
			"							<Param name=\"check_pos_following_error\"/>"
			"							<Param name=\"not_check_pos_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"				<UniqueParam default=\"check_vel\">"
			"					<Param name=\"check_vel\"/>"
			"					<Param name=\"not_check_vel\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_vel_max\">"
			"							<Param name=\"check_vel_max\"/>"
			"							<Param name=\"not_check_vel_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_min\">"
			"							<Param name=\"check_vel_min\"/>"
			"							<Param name=\"not_check_vel_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous\">"
			"							<Param name=\"check_vel_continuous\"/>"
			"							<Param name=\"not_check_vel_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous_at_start\">"
			"							<Param name=\"check_vel_continuous_at_start\"/>"
			"							<Param name=\"not_check_vel_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_following_error\">"
			"							<Param name=\"check_vel_following_error\"/>"
			"							<Param name=\"not_check_vel_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


/*
    struct MoveJRParam
    {
        std::vector<double> begin_pos;
        std::vector<double> target_pos;
        std::vector<double> vel;
        std::vector<double> acc;
        std::vector<double> dec;
        std::vector<bool> joint_active_vec;
    };
    auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        auto c = target.controller;
        MoveJRParam param;
        param.joint_active_vec.resize(target.model->motionPool().size(), false);
        param.begin_pos.resize(target.model->motionPool().size(), false);
        param.target_pos.resize(target.model->motionPool().size(), false);
        param.vel.resize(target.model->motionPool().size(), false);
        param.acc.resize(target.model->motionPool().size(), false);
        param.dec.resize(target.model->motionPool().size(), false);

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "motion_id")
            {
                param.joint_active_vec.resize(target.model->motionPool().size(), false);
                param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
            }
            else if (cmd_param.first == "pos")
            {
                param.target_pos[i] = std::stod(cmd_param.second);
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
        }

        target.param = param;

        std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);
    }
    auto MoveJR::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<MoveJRParam&>(target.param);
        auto controller = target.controller;


        //获取第一个count时，电机的当前角度位置//
        if (target.count == 1)
        {
            for (Size i = 0; i < param.joint_active_vec.size(); ++i)
            {
                if (param.joint_active_vec[i])
                {
                    param.begin_pos = target.controller->motionAtAbs(i).actualPos();
                }
            }
        }

        //梯形轨迹//
        aris::Size total_count{ 1 };
        for (Size i = 0; i < param.joint_active_vec.size(); ++i)
        {
            if (param.joint_active_vec[i])
            {
                double p, v, a;
                aris::Size t_count;
                aris::plan::moveAbsolute(target.count, param.begin_pos[i], param.begin_pos[i] + param.target_pos[i], param.vel[i] / 1000, param.acc[i] / 1000 / 1000, param.dec[i] / 1000 / 1000, p, v, a, t_count);
                controller->motionAtAbs(i).setTargetPos(p);
                //target.model->motionPool().at(i).setMp(p);
                total_count = std::max(total_count, t_count);
            }
        }

        //3D模型同步
        //if (target.model->solverPool().at(1).kinPos())return -1;

        // 打印 //
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					cout << "target_pos" << ":" << param.target_pos[i] << " ";
					cout << "vel" << ":" << param.vel[i] << " ";
					cout << "acc" << ":" << param.acc[i] << " ";
					cout << "dec" << ":" << param.dec[i] << " ";
				}
			}
            for (Size i = 0; i < param.joint_active_vec.size(); ++i)
            {
                if (param.joint_active_vec[i])
                {
                    cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
                    cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                }
            }

            cout << std::endl;
        }

        // log //
        auto &lout = controller->lout();
        for (Size i = 0; i < 6; i++)
        {
            lout << controller->motionAtAbs(i).targetPos() << ",";
            lout << controller->motionAtAbs(i).actualPos() << ",";
            lout << controller->motionAtAbs(i).actualVel() << ",";
        }
        lout << std::endl;

        return total_count - target.count;
    }
    auto MoveJR::collectNrt(PlanTarget &target)->void {}
    MoveJR::MoveJR(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"moveJR\">"
            "	<GroupParam>"
            "		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		<Param name=\"pos\" default=\"0.2\"/>"
            "		<Param name=\"vel\" abbreviation=\"v\" default=\"0.05\"/>"
            "		<Param name=\"acc\" default=\"1\"/>"
            "		<Param name=\"dec\" default=\"1\"/>"
            "		<UniqueParam default=\"check_none\">"
            "			<Param name=\"check_all\"/>"
            "			<Param name=\"check_none\"/>"
            "			<GroupParam>"
            "				<UniqueParam default=\"check_pos\">"
            "					<Param name=\"check_pos\"/>"
            "					<Param name=\"not_check_pos\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_pos_max\">"
            "							<Param name=\"check_pos_max\"/>"
            "							<Param name=\"not_check_pos_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_min\">"
            "							<Param name=\"check_pos_min\"/>"
            "							<Param name=\"not_check_pos_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous\">"
            "							<Param name=\"check_pos_continuous\"/>"
            "							<Param name=\"not_check_pos_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_at_start\">"
            "							<Param name=\"check_pos_continuous_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order\">"
            "							<Param name=\"check_pos_continuous_second_order\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
            "							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_following_error\">"
            "							<Param name=\"check_pos_following_error\"/>"
            "							<Param name=\"not_check_pos_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "				<UniqueParam default=\"check_vel\">"
            "					<Param name=\"check_vel\"/>"
            "					<Param name=\"not_check_vel\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_vel_max\">"
            "							<Param name=\"check_vel_max\"/>"
            "							<Param name=\"not_check_vel_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_min\">"
            "							<Param name=\"check_vel_min\"/>"
            "							<Param name=\"not_check_vel_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous\">"
            "							<Param name=\"check_vel_continuous\"/>"
            "							<Param name=\"not_check_vel_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous_at_start\">"
            "							<Param name=\"check_vel_continuous_at_start\"/>"
            "							<Param name=\"not_check_vel_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_following_error\">"
            "							<Param name=\"check_vel_following_error\"/>"
            "							<Param name=\"not_check_vel_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }

	*/
    struct MoveSineParam
    {
        double begin_pos, target_pos, vel, acc, dec, amp, cycle, phi0, offset;
            int total_time;//持续时间，默认5000count
        std::vector<bool> joint_active_vec;
    };
    auto MoveSine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        auto c = target.controller;
        MoveSineParam param;

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "motion_id")
            {
                param.joint_active_vec.resize(target.model->motionPool().size(), false);
                param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
            }
            else if (cmd_param.first == "pos")
            {
                param.target_pos = std::stod(cmd_param.second);
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
            else if (cmd_param.first == "cycle")
            {
                param.cycle = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "amp")
            {
                param.amp = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "phi0")
            {
                param.phi0 = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "offset")
            {
                param.offset = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "total_time")
            {
                param.total_time = std::stoi(cmd_param.second);
            }
        }

        target.param = param;
        /*
                target.option |=
                    Plan::USE_TARGET_POS;
        */
        /*
                target.option |=
        #ifdef WIN32
                    Plan::NOT_CHECK_POS_MIN |
                    Plan::NOT_CHECK_POS_MAX |
                    Plan::NOT_CHECK_POS_CONTINUOUS |
                    Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
                    Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
                    Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
        #endif
                    Plan::NOT_CHECK_VEL_MIN |
                    Plan::NOT_CHECK_VEL_MAX |
                    Plan::NOT_CHECK_VEL_CONTINUOUS |
                    Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
                    Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
        */
    }
    auto MoveSine::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<MoveSineParam&>(target.param);
        auto controller = target.controller;

        //获取第一个count时，电机的当前角度位置//
        if (target.count == 1)
        {
            for (Size i = 0; i < param.joint_active_vec.size(); ++i)
            {
                if (param.joint_active_vec[i])
                {
                    param.begin_pos = controller->motionAtAbs(i).actualPos();
                }
            }
        }

        //梯形轨迹//
        aris::Size total_count{ 1 };
        for (Size i = 0; i < param.joint_active_vec.size(); ++i)
        {
            if (param.joint_active_vec[i])
            {

                aris::Size t_count;
                param.target_pos = param.begin_pos+param.amp*(1-std::cos(1.0*target.count / param.cycle * 2 * aris::PI))+param.offset;
                controller->motionAtAbs(i).setTargetPos(param.target_pos);

                //aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);*/
                /*controller->motionAtAbs(i).setTargetPos(p);*/
                target.model->motionPool().at(i).setMp(param.target_pos);
                //total_count = std::max(total_count, t_count);
            }
        }
        //3D模型同步
        if (target.model->solverPool().at(1).kinPos())return -1;

        // 打印 //
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec" << ":" << param.dec << " ";

            for (Size i = 0; i < param.joint_active_vec.size(); ++i)
            {
                if (param.joint_active_vec[i])
                {
                    cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
                    cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                    cout << "actualCur" << ":" << controller->motionAtAbs(i).actualCur() << " ";
                }
            }

            cout << std::endl;
        }

        // log //
        auto &lout = controller->lout();
        for (Size i = 0; i < 6; i++)
        {
            lout << controller->motionAtAbs(i).targetPos() << ",";
            lout << controller->motionAtAbs(i).actualPos() << ",";
            lout << controller->motionAtAbs(i).actualVel() << ",";
            lout << controller->motionAtAbs(i).actualCur() << ",";
        }
        lout << std::endl;

        return param.total_time - target.count;
    }
    auto MoveSine::collectNrt(PlanTarget &target)->void {}
    MoveSine::MoveSine(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"moveSine\">"
            "	<GroupParam>"
            "		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		<Param name=\"pos\" default=\"0\"/>"
            "		<Param name=\"vel\" abbreviation=\"v\" default=\"0.08\"/>"
            "		<Param name=\"acc\" default=\"1\"/>"
            "		<Param name=\"dec\" default=\"1\"/>"
            "		<Param name=\"amp\" default=\"0.3\"/>"
            "		<Param name=\"cycle\" default=\"1000\"/>"
            "		<Param name=\"phi0\" default=\"0\"/>"
            "		<Param name=\"offset\" default=\"0\"/>"
            "		<Param name=\"total_time\" default=\"5000\"/>"
            "		<UniqueParam default=\"check_none\">"
            "			<Param name=\"check_all\"/>"
            "			<Param name=\"check_none\"/>"
            "			<GroupParam>"
            "				<UniqueParam default=\"check_pos\">"
            "					<Param name=\"check_pos\"/>"
            "					<Param name=\"not_check_pos\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_pos_max\">"
            "							<Param name=\"check_pos_max\"/>"
            "							<Param name=\"not_check_pos_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_min\">"
            "							<Param name=\"check_pos_min\"/>"
            "							<Param name=\"not_check_pos_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous\">"
            "							<Param name=\"check_pos_continuous\"/>"
            "							<Param name=\"not_check_pos_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_at_start\">"
            "							<Param name=\"check_pos_continuous_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order\">"
            "							<Param name=\"check_pos_continuous_second_order\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
            "							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_following_error\">"
            "							<Param name=\"check_pos_following_error\"/>"
            "							<Param name=\"not_check_pos_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "				<UniqueParam default=\"check_vel\">"
            "					<Param name=\"check_vel\"/>"
            "					<Param name=\"not_check_vel\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_vel_max\">"
            "							<Param name=\"check_vel_max\"/>"
            "							<Param name=\"not_check_vel_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_min\">"
            "							<Param name=\"check_vel_min\"/>"
            "							<Param name=\"not_check_vel_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous\">"
            "							<Param name=\"check_vel_continuous\"/>"
            "							<Param name=\"not_check_vel_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous_at_start\">"
            "							<Param name=\"check_vel_continuous_at_start\"/>"
            "							<Param name=\"not_check_vel_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_following_error\">"
            "							<Param name=\"check_vel_following_error\"/>"
            "							<Param name=\"not_check_vel_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


	// 示教运动--输入末端大地坐标系的位姿pe，控制动作 //
	struct MovePointParam {};
	struct MovePointStruct
	{
		bool movepoint_is_running = false;
		int cor_system;
		int vel_percent;
		std::array<int, 6> is_increase;
	};
	struct MovePoint::Imp
	{
		MovePointStruct s1_rt, s2_nrt;
		std::vector<double> pm_target;
		double vel[6], acc[6], dec[6];
		int increase_count;
	};
	std::atomic_bool movepoint_is_changing = false;
	auto MovePoint::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MovePointParam param;
		imp_->pm_target.resize(16, 0.0);

		std::string ret = "ok";
		target.ret = ret;

		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.movepoint_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.movepoint_is_running = true;
				std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);
				imp_->s2_nrt.cor_system = 0;
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.movepoint_is_running = true;
				std::fill_n(imp_->s1_rt.is_increase.data(), 6, 0);
				imp_->s1_rt.cor_system = 0;
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_AND_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("vel"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->vel);

				mat = target.model->calculator().calculateExpression(params.at("acc"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->acc);

				mat = target.model->calculator().calculateExpression(params.at("dec"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->dec);

				std::fill(target.mot_options.begin(), target.mot_options.end(), USE_TARGET_POS);
				//target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.movepoint_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.movepoint_is_running = false;
				std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
				movepoint_is_changing = true;
				while (movepoint_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			else if (p.first == "cor")
			{
				if (!imp_->s1_rt.movepoint_is_running)throw std::runtime_error("manual mode not started, when pe");

				imp_->s2_nrt.cor_system = std::stoi(params.at("cor"));
				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;
				imp_->s2_nrt.is_increase[0] = std::max(std::min(1, std::stoi(params.at("x"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[1] = std::max(std::min(1, std::stoi(params.at("y"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[2] = std::max(std::min(1, std::stoi(params.at("z"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[3] = std::max(std::min(1, std::stoi(params.at("a"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[4] = std::max(std::min(1, std::stoi(params.at("b"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[5] = std::max(std::min(1, std::stoi(params.at("c"))), -1) * imp_->increase_count;

				imp_->s2_nrt.movepoint_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				movepoint_is_changing = true;
				while (movepoint_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

		target.param = param;
	}
	auto MovePoint::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<MovePointParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };

		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}

		// init status //
		static int increase_status[6]{ 0,0,0,0,0,0 };
		double max_vel[6];
		if (movepoint_is_changing)
		{
			imp_->s1_rt = imp_->s2_nrt;
			movepoint_is_changing.store(false);
			for (int i = 0; i < 6; i++)
			{
				increase_status[i] = imp_->s1_rt.is_increase[i];
			}
			//target.model->generalMotionPool()[0].getMpe(imp_->pe_start, eu_type);
		}

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = imp_->vel[i] * 1.0*imp_->s1_rt.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(increase_status[i])*max_vel[i] * 1e-3;
			increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
		}
		//std::copy_n(target_pos, 6, imp_->pe_start);

		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], imp_->acc[i], imp_->dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		target.model->generalMotionPool()[0].getMpm(pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);
		/*
		s_pe2pm(p_now, imp_->pm_now.data(), eu_type);
		for (int i = 0; i < 6; i++)
		{
			p_now[i] = p_next[i];
			v_now[i] = v_next[i];
			a_now[i] = a_next[i];
		}
		*/

		//绝对坐标系
		if (imp_->s1_rt.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, imp_->pm_target.data());
		}
		//工具坐标系
		else if (imp_->s1_rt.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, imp_->pm_target.data());
		}

		target.model->generalMotionPool().at(0).setMpm(imp_->pm_target.data());

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			cout << "pm_target:" << std::endl;
			for (Size i = 0; i < 16; i++)
			{
				cout << imp_->pm_target[i] << "  ";
			}
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << increase_status[i] << "  ";
			}
			cout << std::endl;
			cout << "p_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_next[i] << "  ";
			}
			cout << std::endl;
			cout << "v_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_next[i] << "  ";
			}
			cout << std::endl;
			cout << "p_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_now[i] << "  ";
			}
			cout << std::endl;
			cout << "v_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_now[i] << "  ";
			}
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.movepoint_is_running ? 1 : 0;
	}
	auto MovePoint::collectNrt(PlanTarget &target)->void {}
	MovePoint::~MovePoint() = default;
	MovePoint::MovePoint(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"movePoint\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
			"				<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.25,0.25,0.25}\"/>"
			"				<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
			"				<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"cor\" default=\"0\"/>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"x\" default=\"0\"/>"
			"				<Param name=\"y\" default=\"0\"/>"
			"				<Param name=\"z\" default=\"0\"/>"
			"				<Param name=\"a\" default=\"0\"/>"
			"				<Param name=\"b\" default=\"0\"/>"
			"				<Param name=\"c\" default=\"0\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	MovePoint::MovePoint(const MovePoint &other) = default;
	MovePoint::MovePoint(MovePoint &other) = default;
	MovePoint& MovePoint::operator=(const MovePoint &other) = default;
	MovePoint& MovePoint::operator=(MovePoint &&other) = default;


	// 示教运动--danzhouguanjie，控制动作 //
	struct MoveJPParam {};
	struct MoveJPStruct
	{
		bool movejp_is_running = false;
		int vel_percent;
		std::vector<int> is_increase;
	};
	struct MoveJP::Imp
	{
		MoveJPStruct s1_rt, s2_nrt;
		double vel, acc, dec;
		std::vector<double> p_now, v_now, a_now, target_pos, max_vel;
		std::vector<int> increase_status;
		int increase_count;
	};
	std::atomic_bool is_changing = false;
	auto MoveJP::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJPParam param;

		std::string ret = "ok";
		target.ret = ret;

		imp_->p_now.resize(c->motionPool().size(), 0.0);
		imp_->v_now.resize(c->motionPool().size(), 0.0);
		imp_->a_now.resize(c->motionPool().size(), 0.0);
		imp_->target_pos.resize(c->motionPool().size(), 0.0);
		imp_->max_vel.resize(c->motionPool().size(), 0.0);
		imp_->increase_status.resize(c->motionPool().size(), 0);

		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.movejp_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.movejp_is_running = true;
				imp_->s2_nrt.is_increase.clear();
				imp_->s2_nrt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.movejp_is_running = true;
				imp_->s1_rt.is_increase.clear();
				imp_->s1_rt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_AND_LINE("");
				imp_->vel = std::stod(params.at("vel"));
				imp_->acc = std::stod(params.at("acc"));
				imp_->dec = std::stod(params.at("dec"));
				std::cout << "prepare2" << std::endl;
				std::fill(target.mot_options.begin(), target.mot_options.end(), NOT_CHECK_POS_FOLLOWING_ERROR | USE_TARGET_POS);
				//target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.movejp_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.movejp_is_running = false;
				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				is_changing = true;
				while (is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
			}
			else if (p.first == "vel_percent")
			{
				if (!imp_->s1_rt.movejp_is_running)throw std::runtime_error("manual mode not started, when pe");

				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;

				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				imp_->s2_nrt.is_increase[std::stoi(params.at("motion_id"))] = std::max(std::min(1, std::stoi(params.at("direction"))), -1) * imp_->increase_count;

				imp_->s2_nrt.movejp_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				is_changing = true;
				while (is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

		target.param = param;
	}
	auto MoveJP::executeRT(PlanTarget &target)->int
	{

		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<MoveJPParam&>(target.param);

		// get current pe //
		if (target.count == 1)
		{
			for (Size i = 0; i < imp_->p_now.size(); ++i)
			{
				/*
				imp_->p_start[i] = target.model->motionPool().at(i).mp();
				imp_->p_now[i] = target.model->motionPool().at(i).mp();
				imp_->v_now[i] = target.model->motionPool().at(i).mv();
				imp_->a_now[i] = target.model->motionPool().at(i).ma();
				*/
				imp_->target_pos[i] = controller->motionAtAbs(i).actualPos();
				imp_->p_now[i] = controller->motionAtAbs(i).actualPos();
				imp_->v_now[i] = controller->motionAtAbs(i).actualVel();
				imp_->a_now[i] = 0.0;
			}
		}
		// init status and calculate target pos and max vel //

		if (is_changing)
		{
			is_changing.store(false);
			imp_->s1_rt = imp_->s2_nrt;
			for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
			{
				imp_->increase_status[i] = imp_->s1_rt.is_increase[i];
			}
		}
		for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
		{
			imp_->max_vel[i] = imp_->vel*1.0*imp_->s1_rt.vel_percent / 100.0;
			imp_->target_pos[i] += aris::dynamic::s_sgn(imp_->increase_status[i])*imp_->max_vel[i] * 1e-3;
			imp_->increase_status[i] -= aris::dynamic::s_sgn(imp_->increase_status[i]);
		}
		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;
		for (int i = 0; i < imp_->p_now.size(); i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(imp_->p_now[i], imp_->v_now[i], imp_->a_now[i]
				, imp_->target_pos[i], 0.0, 0.0
				, imp_->max_vel[i], imp_->acc, imp_->dec
				, 1e-3, 1e-10, p_next, v_next, a_next, t);

			target.model->motionPool().at(i).setMp(p_next);
			controller->motionAtAbs(i).setTargetPos(p_next);
			imp_->p_now[i] = p_next;
			imp_->v_now[i] = v_next;
			imp_->a_now[i] = a_next;
		}

		// 运动学反解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->increase_status[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->target_pos[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->p_now[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->v_now[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < imp_->p_now.size(); i++)
		{
			lout << imp_->target_pos[i] << " ";
			lout << imp_->v_now[i] << " ";
			lout << imp_->a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
			//lout << controller->motionAtAbs(i).actualCur() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.movejp_is_running ? 1 : 0;
	}
	auto MoveJP::collectNrt(PlanTarget &target)->void {}
	MoveJP::~MoveJP() = default;
	MoveJP::MoveJP(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJP\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
			"				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
			"				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
			"				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			"				<Param name=\"direction\" default=\"1\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	MoveJP::MoveJP(const MoveJP &other) = default;
	MoveJP::MoveJP(MoveJP &other) = default;
	MoveJP& MoveJP::operator=(const MoveJP &other) = default;
	MoveJP& MoveJP::operator=(MoveJP &&other) = default;


	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(aris::robot::createPlanRootRokaeXB4());

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
		rs.command().findParam("pos")->setDefaultValue("{0.5,0.39252,0.7899,0.5,0.5,0.5}");

		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::MoveJR>();
        plan_root->planPool().add<kaanh::MoveSine>();
		plan_root->planPool().add<kaanh::MoveJP>();
		plan_root->planPool().add<kaanh::ShowAll>();
		plan_root->planPool().add<kaanh::MovePoint>();
		plan_root->planPool().add<forcecontrol::MoveJRC>();

        plan_root->planPool().add<HomeFix>();

		return plan_root;
	}
	
}
