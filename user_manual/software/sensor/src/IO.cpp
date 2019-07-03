#include<iostream>
#include<aris.hpp>
#include"IO.h"

//using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	

    //添加EtherCAT主站,添加顺序与物理拓扑顺序一致
    controller->slavePool().add<aris::control::EthercatSlave>();
    controller->slavePool().back().setPhyId(1);
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x300);
    //添加第一个EtherCAT从站
    controller->slavePool().add<aris::control::EthercatSlave>();
    controller->slavePool().back().setPhyId(2);
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
    //添加第二个EtherCAT从站
    controller->slavePool().add<aris::control::EthercatSlave>();
    controller->slavePool().back().setPhyId(3);
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
    dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();

	return controller;
};


// 传感器信号测试 //
struct SensorParam
{
    int time;
    float ch1,ch2,ch3,ch4,ch5,ch6;
};
auto Sensor::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        SensorParam param;
        for (auto &p : params)
        {
            if (p.first == "time")
            {
                param.time = std::stoi(p.second);
            }
        }

        target.param = param;
        //将所有option设置为NOT_CHECK_ENABLE
        std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);
    }
auto Sensor::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<FSParam&>(target.param);
        // 访问主站 //
        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

        int16_t rawData;
        float volToSig = 5;
        //主站下标为at(1),从站下标从at(2)开始
        //读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
        //16位精度
        controller->slavePool().at(2).readPdo(0x6000, 0x11, &rawdata, 16);
        param.ch1 = volToSig *rawdata/65535;
        controller->slavePool().at(2).readPdo(0x6010, 0x11, &rawdata, 16);
        param.ch2 = volToSig*rawdata/65535;
        controller->slavePool().at(2).readPdo(0x6020, 0x11, &rawdata, 16);
        param.ch3 = volToSig*rawdata/65535;
        controller->slavePool().at(3).readPdo(0x6000, 0x11, &rawdata, 16);
        param.ch4 = volToSig *rawdata/65535;
        controller->slavePool().at(3).readPdo(0x6010, 0x11, &rawdata, 16);
        param.ch5 = volToSig*rawdata/65535;
        controller->slavePool().at(3).readPdo(0x6020, 0x11, &rawdata, 16);
        param.ch6 = volToSig*rawdata/65535;


        //print//
        //setw(n)设置输出宽度为n
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            cout << std::setw(6) << param.ch1 << "  ";
            cout << std::setw(6) << param.ch2 << "  ";
            cout << std::setw(6) << param.ch3 << "  ";
            cout << std::setw(6) << param.ch4 << "  ";
            cout << std::setw(6) << param.ch5 << "  ";
            cout << std::setw(6) << param.ch6 << "  ";
            cout << std::endl;
            cout << "----------------------------------------------------" << std::endl;
        }

        //log//
        auto &lout = controller->lout();
        {
            lout << param.ch1 << " ";
            lout << param.ch2 << " ";
            lout << param.ch3 << " ";
            lout << param.ch4 << " ";
            lout << param.ch5 << " ";
            lout << param.ch6 << " ";
            lout << std::endl;
        }
        param.time--;
        return param.time;
    }
auto Sensor::collectNrt(PlanTarget &target)->void {}
Sensor::Sensor(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"fssignal\">"
        "	<GroupParam>"
        "		<Param name=\"time\" default=\"100000\"/>"
        "	</GroupParam>"
        "</Command>");
}




// 主函数
int main(int argc, char *argv[])
{
	//创建Ethercat主站对象
    //aris::control::EthercatMaster mst;
	//自动扫描，连接从站
    //mst.scan();
    //std::cout<<mst.xmlString()<<std::endl;

	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());

    std::cout<<"start controller server"<<std::endl;
	//启动线程
	cs.start();
	//getline是将命令行输入的值赋值给command_in
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
