#include <algorithm>
#include"kaanh.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace kaanh
{
    //configuring controller
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.00293480352126769,   -2.50023777179214,   -0.292382537944081,   0.0582675097338009,   1.53363576057128,   26.3545454214145
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 72.857 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 50 / 2 / PI
		};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 130.0 / 360 * 2 * PI,	50.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -84.0 / 360 * 2 * PI, -188.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
			};

			std::string xml_str =
				"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00\""
				" vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
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
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotion>";

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);

#ifndef WIN32
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
	}
		return controller;
};
    //set DH parameters
    auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
        param.d1 = 0.3295;
        param.a1 = 0.04;
        param.a2 = 0.275;
		param.d3 = 0.0;
        param.a3 = 0.025;
        param.d4 = 0.28;

        param.tool0_pe[2] = 0.078;

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

	static double initial[7];
	//末端三点确定圆弧算法
	struct moveC_3PParam
	{
		double End[7], Mid[3];
		double time;
	};
	auto moveC_3P::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		moveC_3PParam param = { { 0.0,0.0,0.0,0.0,0.0,0.0,0.0 },{ 0.0,0.0,0.0},0.0 };
		for (auto &p : params)
		{
			if (p.first == "e1")
			{
				param.End[0] = std::stod(p.second);
			}
			else if (p.first == "e2")
			{
				param.End[1] = std::stod(p.second);
			}
			else if (p.first == "e3")
			{
				param.End[2] = std::stod(p.second);
			}
			else if (p.first == "e4")
			{
				param.End[3] = std::stod(p.second);
			}
			else if (p.first == "e5")
			{
				param.End[4] = std::stod(p.second);
			}
			else if (p.first == "e6")
			{
				param.End[5] = std::stod(p.second);
			}
			else if (p.first == "e7")
			{
				param.End[6] = std::stod(p.second);
			}
			else if (p.first == "m1")
			{
				param.Mid[0] = std::stod(p.second);
			}
			else if (p.first == "m2")
			{
				param.Mid[1] = std::stod(p.second);
			}
			else if (p.first == "m3")
			{
				param.Mid[2] = std::stod(p.second);
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			//用于使用模型轨迹驱动电机//
			Plan::USE_TARGET_POS |
			//Plan::USE_VEL_OFFSET |
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

	}
	auto moveC_3P::executeRT(PlanTarget &target)->int	//每个ms执行一次，target.count计数本函数执行的次数
	{
		auto &ee = target.model->generalMotionPool().at(0);//表示机器人
		auto &param = std::any_cast<moveC_3PParam&>(target.param);//获取参数集，同时把参数集另取一个引用param

		auto time = static_cast<int>(param.time * 1000);
		static double begin_pq[7];
		if (target.count == 1)
		{
			ee.getMpq(initial);
		}

		//对位置进行插值------------------------------------------------------------------------------------------------------------------------------------------
		double pq2[7];
		ee.getMpq(begin_pq);//获取目标：末端的位姿

		//计算 
			// 通过AC=Bb 解出圆心C
		double A[9], Bb[3];
		double C[3];//圆心
		A[0] = 2 * (begin_pq[0] - param.Mid[0]);
		A[1] = 2 * (begin_pq[1] - param.Mid[1]);
		A[2] = 2 * (begin_pq[2] - param.Mid[2]);
		A[3] = 2 * (param.Mid[0] - param.End[0]);
		A[4] = 2 * (param.Mid[1] - param.End[1]);
		A[5] = 2 * (param.Mid[2] - param.End[2]);
		A[6] = A[2] * (A[2] * A[4] - A[1] * A[5]) / 8;
		A[7] = A[2] * (A[0] * A[5] - A[2] * A[3]) / 8;
		A[8] = -(A[0] * (A[2] * A[4] - A[1] * A[5]) + A[1] * (A[0] * A[5] - A[2] * A[3])) / 8;
		Bb[0] = pow(begin_pq[0], 2) + pow(begin_pq[1], 2) + pow(begin_pq[2], 2) - pow(param.Mid[0], 2) - pow(param.Mid[1], 2) - pow(param.Mid[2], 2);
		Bb[1] = pow(param.Mid[0], 2) + pow(param.Mid[1], 2) + pow(param.Mid[2], 2) - pow(param.End[0], 2) - pow(param.End[1], 2) - pow(param.End[2], 2);
		Bb[2] = A[6] * begin_pq[0] + A[7] * begin_pq[1] + A[8] * begin_pq[2];

		//解线性方程组
		double pinv[9];
		std::vector<double> U_vec(9);
		auto U = U_vec.data();
		double tau[3];
		aris::Size p[3];
		aris::Size rank;

		// 根据 A 求出中间变量，相当于做 QR 分解 //
		// 请对 U 的对角线元素做处理
		s_householder_utp(3, 3, A, U, tau, p, rank, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
		double tau2[3];
		s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);
		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //

	//获取圆心
		s_mm(3, 1, 3, pinv, Bb, C);

		//获取半径
		double R = sqrt(pow(begin_pq[0] - C[0], 2) + pow(begin_pq[1] - C[1], 2) + pow(begin_pq[2] - C[2], 2));


		double u, v, w;
		double u1, v1, w1;
		u = (param.Mid[1] - begin_pq[1])*(param.End[2] - param.Mid[2]) - (param.Mid[2] - begin_pq[2])*(param.End[1] - param.Mid[1]);
		v = (param.Mid[2] - begin_pq[2])*(param.End[0] - param.Mid[0]) - (param.Mid[0] - begin_pq[0])*(param.End[2] - param.Mid[2]);
		w = (param.Mid[0] - begin_pq[0])*(param.End[1] - param.Mid[1]) - (param.Mid[1] - begin_pq[1])*(param.End[0] - param.Mid[0]);

		u1 = (begin_pq[1] - C[1])*(param.End[2] - begin_pq[2]) - (begin_pq[2] - C[2])*(param.End[1] - begin_pq[1]);
		v1 = (begin_pq[2] - C[2])*(param.End[0] - begin_pq[0]) - (begin_pq[0] - C[0])*(param.End[2] - begin_pq[2]);
		w1 = (begin_pq[0] - C[0])*(param.End[1] - begin_pq[1]) - (begin_pq[1] - C[1])*(param.End[0] - begin_pq[0]);


		double H = u * u1 + v * v1 + w * w1;
		double theta;
		// 判断theta 与 pi 的关系
		if (H >= 0)
		{
			theta = 2 * asin(sqrt(pow((param.End[0] - begin_pq[0]), 2) + pow((param.End[1] - begin_pq[1]), 2) + pow((param.End[2] - begin_pq[2]), 2)) / (2 * R));
		}
		else
		{
			theta = 2 * 3.1415 - 2 * asin(sqrt(pow((param.End[0] - begin_pq[0]), 2) + pow((param.End[1] - begin_pq[1]), 2) + pow((param.End[2] - begin_pq[2]), 2)) / (2 * R));
		}
		//得到迭代点的位置信息
		double delta = theta / time;
		if (time <= 4)
		{
			std::cout << "parameter time is too small, please try to use bigger one" << std::endl;
			return 0;
		}
		double FT = R * tan(delta);
		double  G = 1 / sqrt(1 + pow(FT / R, 2));
		double  E = FT / (R*sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2)));
		double m, n, l;
		m = v * (begin_pq[2] - C[2]) - w * (begin_pq[1] - C[1]);
		n = w * (begin_pq[0] - C[0]) - u * (begin_pq[2] - C[2]);
		l = u * (begin_pq[1] - C[1]) - v * (begin_pq[0] - C[0]);

		pq2[0] = C[0] + G * (begin_pq[0] + E * m - C[0]);
		pq2[1] = C[1] + G * (begin_pq[1] + E * n - C[1]);
		pq2[2] = C[2] + G * (begin_pq[2] + E * l - C[2]);

		//对姿态进行插值--------------------------------------------------------------------------------------------------------------------------------------------------------------------
		float t = target.count*1.0 / time;
		float cosa = initial[3] * param.End[3] + initial[4] * param.End[4] + initial[5] * param.End[5] + initial[6] * param.End[6];
		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
		// the shorter path. Fix by reversing one quaternion.
		if (cosa < 0.0f)
		{
			param.End[3] = -param.End[3];
			param.End[4] = -param.End[4];
			param.End[5] = -param.End[5];
			param.End[6] = -param.End[6];
			cosa = -cosa;
		}

		double k0, k1;
		// If the inputs are too close for comfort, linearly interpolate
		if (cosa > 0.9995f)
		{
			k0 = 1.0f - t;
			k1 = t;
		}
		else
		{
			float sina = sqrt(1.0f - cosa * cosa);
			float a = atan2(sina, cosa);
			k0 = sin((1.0f - t)*a) / sina;
			k1 = sin(t *a) / sina;
		}
		pq2[3] = initial[3] * k0 + param.End[3] * k1;
		pq2[4] = initial[4] * k0 + param.End[4] * k1;
		pq2[5] = initial[5] * k0 + param.End[5] * k1;
		pq2[6] = initial[6] * k0 + param.End[6] * k1;


		ee.setMpq(pq2);//让目标：末端，执行到pq2的位姿

		if (target.model->solverPool().at(0).kinPos())return -1;
		target.model->solverPool().at(0).kinVel();

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			cout << "cur:" << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;
			cout << "R=" << "  " << R << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (int i = 0; i < 7; i++)
		{
			lout << pq2[i] << " ";
		}
		lout << std::endl;

		return time - target.count;
	}
	auto moveC_3P::collectNrt(PlanTarget &target)->void {}
	moveC_3P::moveC_3P(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveC_3P\">"
			"	<GroupParam>"
			//			"		<UniqueParam default=\"x\">"
			"		<Param name=\"e1\" default=\"0.45\"/>"        // b1 b2 b3 是末点坐标
			"		<Param name=\"e2\" default=\"0\"/>"
			"		<Param name=\"e3\" default=\"0.75\"/>"
			"		<Param name=\"e4\" default=\"1\"/>"
			"		<Param name=\"e5\" default=\"-0.5\"/>"
			"		<Param name=\"e6\" default=\"0\"/>"
			"		<Param name=\"e7\" default=\"1\"/>"
			"		<Param name=\"m1\" default=\"0.37\"/>"        // p1 p2 p3 是中间点坐标
			"		<Param name=\"m2\" default=\"0\"/>"
			"		<Param name=\"m3\" default=\"0.8\"/>"

			//			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"3.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}



	//末端圆弧轨迹_给定圆心、末点算法

	struct moveC_CEParam
	{
		double End[3], O[3];
		double time;
	};
	auto moveC_CE::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		moveC_CEParam param = { { 0.0,0.0,0.0 },{ 0.0,0.0,0.0 },0.0 };
		for (auto &p : params)
		{
			if (p.first == "e1")
			{
				param.End[0] = std::stod(p.second);
			}
			else if (p.first == "e2")
			{
				param.End[1] = std::stod(p.second);
			}
			else if (p.first == "e3")
			{
				param.End[2] = std::stod(p.second);
			}
			else if (p.first == "o1")
			{
				param.O[0] = std::stod(p.second);
			}
			else if (p.first == "o2")
			{
				param.O[1] = std::stod(p.second);
			}
			else if (p.first == "o3")
			{
				param.O[2] = std::stod(p.second);
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			//用于使用模型轨迹驱动电机//
			Plan::USE_TARGET_POS |
			//Plan::USE_VEL_OFFSET |
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

	}
	auto moveC_CE::executeRT(PlanTarget &target)->int	//每个ms执行一次，target.count计数本函数执行的次数
	{
		auto &ee = target.model->generalMotionPool().at(0);//表示机器人
		auto &param = std::any_cast<moveC_CEParam&>(target.param);//获取参数集，同时把参数集另取一个引用param

		auto time = static_cast<int>(param.time * 1000);
		static double begin_pq[7];
		if (target.count == 1)
		{
			ee.getMpq(begin_pq);
		}
		double pq2[7];
		double pqv[7] = { 0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0 };
		ee.getMpq(pq2);//获取目标：末端的位姿

		//计算
		double AE[3];
		AE[0] = param.End[0] - begin_pq[0];
		AE[1] = param.End[1] - begin_pq[1];
		AE[2] = param.End[2] - begin_pq[2];

		double OA[3];
		OA[0] = begin_pq[0] - param.O[0];
		OA[1] = begin_pq[1] - param.O[1];
		OA[2] = begin_pq[2] - param.O[2];

		double d, R, R1;
		d = sqrt(pow(AE[0], 2) + pow(AE[1], 2) + pow(AE[2], 2));
		R = sqrt(pow(OA[0], 2) + pow(OA[1], 2) + pow(OA[2], 2));
		R1 = sqrt(pow(OA[0] + AE[0], 2) + pow(OA[1] + AE[1], 2) + pow(OA[2] + AE[2], 2));


		if (abs(R - R1) > 0.1)
		{
			std::cout << "wrong input parameter" << std::endl;
			std::cout << "d=" << d << std::endl;
			std::cout << "R=" << R << std::endl;
			std::cout << "R1=" << R1 << std::endl;

			for (int i = 0; i < 3; i++)
			{
				std::cout << "A[" << i << "]=" << begin_pq[i] << std::endl;
			}
			return 0;
		}
		double theta = 2 * asin(d / (2 * R));
		double a1 = sin(theta - theta * target.count / time) / sin(theta), a2 = sin(theta * target.count / time) / sin(theta);

		pq2[0] = begin_pq[0] + (a1 + a2 - 1)*OA[0] + a2 * AE[0];
		pq2[1] = begin_pq[1] + (a1 + a2 - 1)*OA[1] + a2 * AE[1];
		pq2[2] = begin_pq[2] + (a1 + a2 - 1)*OA[2] + a2 * AE[2];
		ee.setMpq(pq2);//让目标：末端，执行到pq2的位姿

		if (target.model->solverPool().at(0).kinPos())return -1;
		target.model->solverPool().at(0).kinVel();

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "cur:" << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		lout << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;

		return time - target.count;
	}
	auto moveC_CE::collectNrt(PlanTarget &target)->void {}
	moveC_CE::moveC_CE(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveC_CE\">"
			"	<GroupParam>"
			//			"		<UniqueParam default=\"x\">"
			"		<Param name=\"e1\" default=\"0.398\"/>"
			"		<Param name=\"e2\" default=\"0\"/>"
			"		<Param name=\"e3\" default=\"0.789\"/>"
			"		<Param name=\"o1\" default=\"0.37\"/>"
			"		<Param name=\"o2\" default=\"0\"/>"
			"		<Param name=\"o3\" default=\"0.709\"/>"

			//			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"3.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}

	// 末端圆弧轨迹
	struct moveC_REParam
	{
		double End[3], V[3];
		double time, R;
	};
	auto moveC_RE::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		moveC_REParam param = { { 0.0,0.0,0.0 },0.0,0.0 };
		for (auto &p : params)
		{

			if (p.first == "e1")
			{
				param.End[0] = std::stod(p.second);
			}
			else if (p.first == "e2")
			{
				param.End[1] = std::stod(p.second);
			}
			else if (p.first == "e3")
			{
				param.End[2] = std::stod(p.second);
			}
			else if (p.first == "v1")
			{
				param.V[0] = std::stod(p.second);
			}
			else if (p.first == "v2")
			{
				param.V[1] = std::stod(p.second);
			}
			else if (p.first == "v3")
			{
				param.V[2] = std::stod(p.second);
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
			else if (p.first == "R")
			{
				param.R = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			//用于使用模型轨迹驱动电机//
			Plan::USE_TARGET_POS |
			//Plan::USE_VEL_OFFSET |
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

	}
	auto moveC_RE::executeRT(PlanTarget &target)->int	//每个ms执行一次，target.count计数本函数执行的次数
	{
		auto &ee = target.model->generalMotionPool().at(0);//表示机器人
		auto &param = std::any_cast<moveC_REParam&>(target.param);//获取参数集，同时把参数集另取一个引用param

		auto time = static_cast<int>(param.time * 1000);
		static double begin_pq[7];
		if (target.count == 1)
		{
			ee.getMpq(begin_pq);
		}
		double pq2[7];
		double pqv[7] = { 0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0 };
		ee.getMpq(pq2);//获取目标：末端的位姿
		// 计算
		double AE[3];
		AE[0] = param.End[0] - begin_pq[0];
		AE[1] = param.End[1] - begin_pq[1];
		AE[2] = param.End[2] - begin_pq[2];

		double d = sqrt(pow(AE[0], 2) + pow(AE[1], 2) + pow(AE[2], 2));

		double theta = 2 * asin(d / (2 * param.R));
		double a1 = sin(theta - theta * target.count / time) / sin(theta), a2 = sin(theta * target.count / time) / sin(theta);

		double g = AE[0] * param.V[0] + AE[1] * param.V[1] + AE[2] * param.V[2];

		double OA[3];
		double d1 = sqrt(pow(param.R, 2) - pow(d, 2) / 4) / (d*sqrt(pow(d, 2)*(pow(param.V[0], 2) + pow(param.V[1], 2) + pow(param.V[2], 2)) - pow(g, 2)));
		OA[0] = -AE[0] / 2 + (g* AE[0] - pow(d, 2)*param.V[0])*d1;
		OA[1] = -AE[0] / 2 + (g* AE[1] - pow(d, 2)*param.V[1])*d1;
		OA[2] = -AE[0] / 2 + (g* AE[2] - pow(d, 2)*param.V[2])*d1;
		pq2[0] = begin_pq[0] + (a1 + a2 - 1)*OA[0] + a2 * (AE[0]);
		pq2[1] = begin_pq[1] + (a1 + a2 - 1)*OA[1] + a2 * (AE[1]);
		pq2[2] = begin_pq[2] + (a1 + a2 - 1)*OA[2] + a2 * (AE[2]);
		ee.setMpq(pq2);//让目标：末端，执行到pq2的位姿

		if (target.model->solverPool().at(0).kinPos())return -1;
		target.model->solverPool().at(0).kinVel();

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "cur:" << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		lout << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;

		return time - target.count;
	}
	auto moveC_RE::collectNrt(PlanTarget &target)->void {}
	moveC_RE::moveC_RE(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveC_RE\">"
			"	<GroupParam>"
			//			"		<UniqueParam default=\"x\">"

			"		<Param name=\"e1\" default=\"0.5\"/>"
			"		<Param name=\"e2\" default=\"0.39\"/>"
			"		<Param name=\"e3\" default=\"0.79\"/>"
			"		<Param name=\"v1\" default=\"0\"/>"
			"		<Param name=\"v2\" default=\"0\"/>"
			"		<Param name=\"v3\" default=\"1\"/>"
			"		<Param name=\"R\" default=\"0.3\"/>"
			//			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"4.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}






	// 末端四元数xyz方向余弦轨迹；速度前馈//
	struct moveL_CosParam
	{
		double x, y, z;
		double time;
	};
	auto moveL_Cos::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		moveL_CosParam param = { 0.0,0.0,0.0,0.0 };
		for (auto &p : params)
		{
			if (p.first == "x")
			{
				param.x = std::stod(p.second);
			}
			else if (p.first == "y")
			{
				param.y = std::stod(p.second);
			}
			else if (p.first == "z")
			{
				param.z = std::stod(p.second);
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			//用于使用模型轨迹驱动电机//
			Plan::USE_TARGET_POS |
			//Plan::USE_VEL_OFFSET |
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

	}
	static double begin_pq1[7];
	auto moveL_Cos::executeRT(PlanTarget &target)->int	//每个ms执行一次，target.count计数本函数执行的次数
	{
		auto &ee = target.model->generalMotionPool().at(0);//表示机器人
		auto &param = std::any_cast<moveL_CosParam&>(target.param);//获取参数集，同时把参数集另取一个引用param

		auto time = static_cast<int>(param.time * 1000);
		if (target.count == 1)
		{
			ee.getMpq(begin_pq1);
		}
		double pq2[7];
		double pqv[7] = { 0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0 };
		ee.getMpq(pq2);//获取目标：末端的位姿

		if ((int)floor(2.0*target.count / time) % 2 == 0)
		{
			for (Size i = 0; i < 3; ++i)
			{
				pq2[i] = begin_pq1[i] + param.x*(1 - std::cos(4 * PI*target.count / time)) / 2;
			}
		}
		else
		{
			for (Size i = 0; i < 3; ++i)
			{
				pq2[i] = begin_pq1[i] - param.x*(1 - std::cos(4 * PI*target.count / time)) / 2;
			}
		}

		ee.setMpq(pq2);//让目标：末端，执行到pq2的位姿

		if (target.model->solverPool().at(0).kinPos())return -1;

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "cur:" << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		lout << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;

		return time - target.count;
	}
	auto moveL_Cos::collectNrt(PlanTarget &target)->void {}
	moveL_Cos::moveL_Cos(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveL_Cos\">"
			"	<GroupParam>"
			//			"		<UniqueParam default=\"x\">"
			"		<Param name=\"x\" default=\"0.02\"/>"
			"		<Param name=\"y\" default=\"0.0\"/>"
			"		<Param name=\"z\" default=\"0.0\"/>"
			//			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"3.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//末端直线梯形轨迹规划
	struct moveL_TParam
	{
		double dpm_pq[3];
		double vel, acc, dec;
	};
	auto moveL_T::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		moveL_TParam param;

		for (auto cmd_param : params)
		{

			if (cmd_param.first == "dx")
			{
				param.dpm_pq[0] = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dy")
			{
				param.dpm_pq[1] = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dz")
			{
				param.dpm_pq[2] = std::stod(cmd_param.second);
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

		target.option |=
			//				Plan::USE_TARGET_POS |
			Plan::USE_VEL_OFFSET |
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

	}
	static double begin_pq[7];
	auto moveL_T::executeRT(PlanTarget &target)->int
	{
		auto &ee = target.model->generalMotionPool().at(0);//表示机器人
		auto &param = std::any_cast<moveL_TParam&>(target.param);
		auto controller = target.controller;

		if (target.count == 1)
		{
			ee.getMpq(begin_pq);
		}
		double pq2[7];
		pq2[3] = begin_pq[3];
		pq2[4] = begin_pq[4];
		pq2[5] = begin_pq[5];
		pq2[6] = begin_pq[6];

		aris::Size total_count{ 1 };
		for (Size i = 0; i < 3; ++i)
		{
			double p, v, a;
			aris::Size t_count;
			aris::plan::moveAbsolute(target.count, begin_pq[i], begin_pq[i] + param.dpm_pq[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
			pq2[i] = p;
			total_count = std::max(total_count, t_count);

		}
		ee.setMpq(pq2);

		//controller与模型同步，保证3D仿真模型同步显示
		if (target.model->solverPool().at(0).kinPos())return -1;// at(0)反解， at(1)正解


		return total_count - target.count;
	}
	auto moveL_T::collectNrt(PlanTarget &target)->void {}
	moveL_T::moveL_T(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveL_T\">"
			"	<GroupParam>"
			"		<Param name=\"dx\" default=\"0.1\"/>"
			"		<Param name=\"dy\" default=\"0.1\"/>"
			"		<Param name=\"dz\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.5\"/>"
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


	// 单关节正弦往复轨迹 //
	struct moveJ_CosParam
	{
		double j[6];
		double time;
		uint32_t timenum;
		std::vector<bool> joint_active_vec;
	};
	auto moveJ_Cos::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		//moveJ_CosParam param = {{0.0,0.0,0.0,0.0,0.0,0.0},0.0,0};
		moveJ_CosParam param;
		for (Size i = 0; i < 6; i++)
		{
			param.j[i] = 0.0;
		}
		param.time = 0.0;
		param.timenum = 0;

		param.joint_active_vec.clear();
		param.joint_active_vec.resize(target.model->motionPool().size(), true);

		for (auto &p : params)
		{
			if (p.first == "j1")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[0] = false;
					param.j[0] = target.model->motionPool()[0].mp();
				}
				else
				{
					param.joint_active_vec[0] = true;
					param.j[0] = std::stod(p.second);
				}

			}
			else if (p.first == "j2")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[1] = false;
					param.j[1] = target.model->motionPool()[1].mp();
				}
				else
				{
					param.joint_active_vec[1] = true;
					param.j[1] = std::stod(p.second);
				}
			}
			else if (p.first == "j3")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[2] = false;
					param.j[2] = target.model->motionPool()[2].mp();
				}
				else
				{
					param.joint_active_vec[2] = true;
					param.j[2] = std::stod(p.second);
				}
			}
			else if (p.first == "j4")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[3] = false;
					param.j[3] = target.model->motionPool()[3].mp();
				}
				else
				{
					param.joint_active_vec[3] = true;
					param.j[3] = std::stod(p.second);
				}
			}
			else if (p.first == "j5")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[4] = false;
					param.j[4] = target.model->motionPool()[4].mp();
				}
				else
				{
					param.joint_active_vec[4] = true;
					param.j[4] = std::stod(p.second);
				}
			}
			else if (p.first == "j6")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[5] = false;
					param.j[5] = target.model->motionPool()[5].mp();
				}
				else
				{
					param.joint_active_vec[5] = true;
					param.j[5] = std::stod(p.second);
				}
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
			else if (p.first == "timenum")
			{
				param.timenum = std::stoi(p.second);
			}
		}
		target.param = param;

		target.option |=
			Plan::USE_TARGET_POS |
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

	}
	auto moveJ_Cos::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<moveJ_CosParam&>(target.param);
		auto time = static_cast<int32_t>(param.time * 1000);
		auto totaltime = static_cast<int32_t>(param.timenum * time);
		static double begin_pjs[6];
		static double step_pjs[6];



		// 获取当前起始点位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				begin_pjs[i] = target.model->motionPool()[i].mp();
				step_pjs[i] = target.model->motionPool()[i].mp();
			}
		}
		if ((int)floor(2.0*target.count / time) % 2 == 0)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				step_pjs[i] = begin_pjs[i] + param.j[i] * (1 - std::cos(4 * PI*target.count / time)) / 2;
				target.model->motionPool().at(i).setMp(step_pjs[i]);
			}
		}
		else
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				step_pjs[i] = begin_pjs[i] - param.j[i] * (1 - std::cos(4 * PI*target.count / time)) / 2;
				target.model->motionPool().at(i).setMp(step_pjs[i]);
			}
		}



		if (target.model->solverPool().at(1).kinPos())return -1;

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return totaltime - target.count;
	}
	auto moveJ_Cos::collectNrt(PlanTarget &target)->void {}
	moveJ_Cos::moveJ_Cos(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJ_Cos\">"
			"	<GroupParam>"
			"		<Param name=\"j1\" default=\"current_pos\"/>"
			"		<Param name=\"j2\" default=\"current_pos\"/>"
			"		<Param name=\"j3\" default=\"current_pos\"/>"
			"		<Param name=\"j4\" default=\"current_pos\"/>"
			"		<Param name=\"j5\" default=\"current_pos\"/>"
			"		<Param name=\"j6\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}



	// 单关节相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct moveJ_TParam
	{
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto moveJ_T::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		moveJ_TParam param;

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
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		target.option |=
			//				Plan::USE_TARGET_POS |
			Plan::USE_VEL_OFFSET |
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

	}
	auto moveJ_T::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<moveJ_TParam&>(target.param);
		auto controller = target.controller;

		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_joint_pos_vec[i] = controller->motionAtAbs(i).actualPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_joint_pos_vec[i], param.begin_joint_pos_vec[i] + param.joint_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				controller->motionAtAbs(i).setTargetVel(v * 1000);
				total_count = std::max(total_count, t_count);

				target.model->motionPool().at(i).setMp(p);
			}
		}

		//controller与模型同步，保证3D仿真模型同步显示
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "mp" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).targetPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto moveJ_T::collectNrt(PlanTarget &target)->void {}
	moveJ_T::moveJ_T(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJ_T\">"
			"	<GroupParam>"
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



	// 多关节混合插值梯形轨迹；速度前馈 //
	struct moveJM_TParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		bool ab;
	};
	auto moveJM_T::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		moveJM_TParam param;
		param.total_count_vec.resize(6, 1);
		param.axis_begin_pos_vec.resize(6, 0.0);

		//params.at("pos")
		for (auto &p : params)
		{
			if (p.first == "pos")
			{
				if (p.second == "current_pos")
				{
					target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				else
				{
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						//超阈值保护//
						if (param.axis_pos_vec[i] > 1.0)
						{
							param.axis_pos_vec[i] = 1.0;
						}
						if (param.axis_pos_vec[i] < -1.0)
						{
							param.axis_pos_vec[i] = -1.0;
						}
						if (param.axis_pos_vec[i] >= 0)
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
						}
						else
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
						}
					}
				}
			}
			else if (p.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
				}
				else if (v.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
					//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					if (param.axis_vel_vec[i] > 1.0)
					{
						param.axis_vel_vec[i] = 1.0;
					}
					if (param.axis_vel_vec[i] < 0.0)
					{
						param.axis_vel_vec[i] = 0.0;
					}
					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (p.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
				}
				else if (a.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0)
					{
						param.axis_acc_vec[i] = 1.0;
					}
					if (param.axis_acc_vec[i] < 0.0)
					{
						param.axis_acc_vec[i] = 0.0;
					}
					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (p.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
				}
				else if (d.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0)
					{
						param.axis_dec_vec[i] = 1.0;
					}
					if (param.axis_dec_vec[i] < 0.0)
					{
						param.axis_dec_vec[i] = 0.0;
					}
					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
			else if (p.first == "ab")
			{
				param.ab = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			//Plan::USE_VEL_OFFSET |
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

	}
	auto moveJM_T::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<moveJM_TParam&>(target.param);
		static double begin_pos[6];
		static double pos[6];
		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
			}
		}
		// 设置驱动器的位置 //
		if (param.ab)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v * 1000);
				target.model->motionPool().at(i).setMp(p);
			}
		}
		else
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v * 1000);
				target.model->motionPool().at(i).setMp(p);
			}
		}

		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	auto moveJM_T::collectNrt(PlanTarget &target)->void {}
	moveJM_T::moveJM_T(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM_T\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"		<Param name=\"ab\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 关节插值运动轨迹--输入末端pq姿态，各个关节的速度、加速度；各关节按照梯形速度轨迹执行；速度前馈 //
	struct moveJM_TQParam
	{
		std::vector<double> pq;
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto moveJM_TQ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		moveJM_TQParam param;
		param.pq.resize(7, 0.0);
		param.total_count_vec.resize(6, 1);
		param.axis_begin_pos_vec.resize(6, 0.0);
		param.axis_pos_vec.resize(6, 0.0);

		//params.at("pq")
		for (auto &p : params)
		{
			if (p.first == "pq")
			{
				if (p.second == "current_pos")
				{
					target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				else
				{
					auto pqarray = target.model->calculator().calculateExpression(p.second);
					param.pq.assign(pqarray.begin(), pqarray.end());
				}
			}
			else if (p.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(param.axis_pos_vec.size(), v.toDouble());
				}
				else if (v.size() == param.axis_pos_vec.size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
				{
					//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
					//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					if (param.axis_vel_vec[i] > 1.0)
					{
						param.axis_vel_vec[i] = 1.0;
					}
					if (param.axis_vel_vec[i] < 0.0)
					{
						param.axis_vel_vec[i] = 0.0;
					}
					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (p.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(param.axis_pos_vec.size(), a.toDouble());
				}
				else if (a.size() == param.axis_pos_vec.size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0)
					{
						param.axis_acc_vec[i] = 1.0;
					}
					if (param.axis_acc_vec[i] < 0.0)
					{
						param.axis_acc_vec[i] = 0.0;
					}
					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (p.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(param.axis_pos_vec.size(), d.toDouble());
				}
				else if (d.size() == param.axis_pos_vec.size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0)
					{
						param.axis_dec_vec[i] = 1.0;
					}
					if (param.axis_dec_vec[i] < 0.0)
					{
						param.axis_dec_vec[i] = 0.0;
					}
					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
		}
		target.param = param;

		target.option |=
			Plan::USE_VEL_OFFSET |
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

	}
	auto moveJM_TQ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<moveJM_TQParam&>(target.param);
		static double begin_pos[6];
		static double pos[6];
		// 取得起始位置 //
		if (target.count == 1)
		{
			target.model->generalMotionPool().at(0).setMpq(param.pq.data());	//generalMotionPool()指模型末端，at(0)表示第1个末端，对于6足就有6个末端，对于机器人只有1个末端
			if (target.model->solverPool().at(0).kinPos())return -1;
			for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
				param.axis_pos_vec[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
			}
		}
		// 设置驱动器的位置 //
		for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
		{
			double p, v, a;
			aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
				, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
			controller->motionAtAbs(i).setTargetPos(p);
			//速度前馈//
			controller->motionAtAbs(i).setOffsetVel(v * 1000);
			target.model->motionPool().at(i).setMp(p);
		}
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	auto moveJM_TQ::collectNrt(PlanTarget &target)->void {}
	moveJM_TQ::moveJM_TQ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM_TQ\">"
			"	<GroupParam>"
			"		<Param name=\"pq\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.05,0.05,0.05}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
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
		for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
		{
			param.axis_pos_vec[i] = controller->motionAtAbs(i).actualPos();
		}

        target.model->generalMotionPool().at(0).getMpq(param.axis_pq_vec.data());

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

        param.joint_active_vec.clear();

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

        //std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
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
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}
		//3D模型同步
        if (target.model->solverPool().at(1).kinPos())return -1;

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

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
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
                std::cout<<"prepare2"<<std::endl;
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
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.39252,0.7899,0.5,0.5,0.5}");

		plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::GetPartPq>();
        plan_root->planPool().add<aris::plan::SetXml>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::MoveJP>();
		plan_root->planPool().add<kaanh::ShowAll>();
		plan_root->planPool().add<kaanh::MovePoint>();


		plan_root->planPool().add<kaanh::moveC_3P>();
		plan_root->planPool().add<kaanh::moveC_CE>();
		plan_root->planPool().add<kaanh::moveC_RE>();
		plan_root->planPool().add<kaanh::moveL_Cos>();
		plan_root->planPool().add<kaanh::moveL_T>();
		plan_root->planPool().add<kaanh::moveJ_Cos>();
		plan_root->planPool().add<kaanh::moveJ_T>();
		plan_root->planPool().add<kaanh::moveJM_T>();
		plan_root->planPool().add<kaanh::moveJM_TQ>();
		return plan_root;
	}
	
}
