#include <iostream>
//程序引用aris库的头文件
#include <aris.hpp>
#include<string>


int main()
{
	std::cout << "start testing IO board" << std::endl;

	aris::control::EthercatMaster mst;	//创建EtherCAT主站对象	
    auto&cs = aris::server::ControlServer::instance();
	//手动配置从站，从站的顺序程序添加的顺序一致，本例采用xml格式，配置了2个EtherCAT从站，第一个从站是EtherCAT IO板卡、第二个从站是松下伺服驱动器
	//参考从站ESI文件，添加从站——EtherCAT IO板卡配置
	//xml_str中,size的单位是bit
	std::string xml_str =
		"<ethercatIO type=\"EthercatSlave\" phy_id=\"1\" product_code=\"0x00201\""
		" vendor_id=\"0x00000A09\" revision_num=\"0x64\" dc_assign_activate=\"0x00\">"
		"	<sm_pool type=\"SyncManagerPoolObject\">"
		"		<sm type=\"SyncManager\" is_tx=\"false\">"
		"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
		"				<Dout_0_7 index=\"0x7001\" subindex=\"0x01\" size=\"8\"/>"
		"			</index_1600>"
		"		</sm>"
		"		<sm type=\"SyncManager\" is_tx=\"false\">"
		"			<index_1601 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1601\" is_tx=\"false\">"
		"				<Dout_8_15 index=\"0x7001\" subindex=\"0x02\" size=\"8\"/>"
		"			</index_1601>"
		"		</sm>"
		"		<sm type=\"SyncManager\" is_tx=\"true\">"
		"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1a00\" is_tx=\"true\">"
		"				<Din_0_7 index=\"0x6001\" subindex=\"0x01\" size=\"8\"/>"
		"			</index_1a00>"
		"			<index_1a01 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1a01\" is_tx=\"true\">"
		"				<Din_8_15 index=\"0x6001\" subindex=\"0x02\" size=\"8\"/>"
		"			</index_1a01>"
		"		</sm>"
		"	</sm_pool>"
		"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
		"	</sdo_pool>"
		"</ethercatIO>";
	//添加EtherCAT从站。其中，
	//(1)成员函数SlavePool()返回从站的引用，类型为vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()
	//(2)vector的类型是aris::control::EthercatSlave，函数add()实现添加vector成员
	//(3)函数loadXmlStr()实现加载从站配置信息xml_str
	mst.slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);
	
	//参考ESI文件，添加从站——松下伺服驱动器的配置
	xml_str =
		"<m_servo type=\"EthercatSlave\" phy_id=\"0\" product_code=\"0x60380007\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\">"
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
		"</m_servo>";
	//添加EtherCAT从站。其中，
	//(1)成员函数SlavePool()返回从站的引用，类型为vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()
	//(2)vector的类型是aris::control::EthercatSlave，函数add()实现添加vector成员
	//(3)函数loadXmlStr()实现加载从站配置信息xml_str
	mst.slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);

	
	//自动扫描、连接从站(默认是注释掉mst.scan())，从站的顺序与实际物理拓扑相同。本例中，物理拓扑第一个从站是松下驱动器，第二个从站是EtherCAT IO板卡；如果使用自动扫从站模式，mst.ecSlavePool().at(0)需要改成mst.ecSlavePool().at(1);
	//mst.scan();
	
	//打印主站扫描的从站个数
	std::cout << "slave num:" << mst.slavePool().size() << std::endl;		

	//主站对象mst的成员函数setControlStrategy()创建一个实时线程。其形参即是被调用的实时函数，在实时核中每1ms被调用1次
	//被调用函数可以实现主站与从站之间的数据交互，打印服务，log服务。(本例以读、写IO信号，并打印为例)
	mst.setControlStrategy([&]()
	{
		static int count{ 0 };
		static std::uint8_t value_dq{ 0x01 };
		static std::uint8_t value_di{ 0x00 };
		//
		if (++count % 1000 == 0)
		{
			//EtherCAT IO板卡输出口实现“走马灯”
			value_dq = value_dq << 1;
			if (value_dq == 0) value_dq = 0x01;

			//成员函数mout()是实时核打印函数接口，成员函数lout()是实时核log函数接口
			mst.mout() << "count:" << std::dec << count << std::endl;
			mst.lout() << "count:" << std::dec << count << std::endl;

			//成员函数ecSlavePool()创建从站vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()，at(0)表示第一个从站，即EtherCAT IO板卡
			mst.slavePool().at(0).writePdo(0x7001, 0x02, &value_dq, 8);	//writePdo是写函数，第一个形参是index，第二个形参是subindex，第三个形参写DO的数值，第四个形参表示写操作的bit数
			mst.slavePool().at(0).readPdo(0x6001, 0x01, &value_di, 8);	//readPdo是读函数，第一个形参是index，第二个形参是subindex，第三个形参读DI的数值，第四个形参表示读操作的bit数
		}
	});

	//启动实时线程
	cs.start();

	//非实时线程睡眠100秒钟
	std::this_thread::sleep_for(std::chrono::seconds(100));

	//关闭实时线程
	cs.stop();

	return 0;
}
