#include "StdAfx.h"
#include "NavigationProcess.h"



CNavigationProcess::CNavigationProcess(void)
	: M_pRangeData(new pcl::PointCloud<pcl::PointXYZ>)
	, M_pVelodyneRangeData(new pcl::PointCloud<pcl::PointXYZI>)
{
	std::cout << "[CNavigationProcess]: Instance is created!!!" << endl;
}


CNavigationProcess::~CNavigationProcess(void)
{
	std::cout << "[CNavigationProcess]: Instance is destroyed!!!" << endl;
}


void CNavigationProcess::Thread(void* arg)
{
	CNavigationProcess * pProc = (CNavigationProcess *)arg;

	switch (pProc->m_nProcMode)
	{
	case BASIC: pProc->doProcess(); break;
	case PRACTICE: pProc->doProcess_Practice(); break;
	case SIMULATION: pProc->doProcess_Simulation(); break;
	case TEST: pProc->doProcess_Test(); break;
	}
}

void CNavigationProcess::initialize()
{
	switch (m_nProcMode)
	{
	case TEST:
	{
		CSensorDataManager::getInstance()->setSensorUsingFlag(CSensorDataManager::PIONEER3DX, true);

		M_RobotState = CRenderingObjects::getInstance()->getRobotState();
		M_GoalState = CRenderingObjects::getInstance()->getGoalState();

		CPioneer3DXInterface::getInstance()->setMaxVelocity(0.25, 40.0*D2R);			//20181215
		M_pLRFData = new double[181];
		for (int i = 0; i < 181; i++)
			M_pLRFData[i] = 15.0;
		//CPioneer3DXInterface::getInstance()->setMaxVelocity(0.5, 80.0*D2R );

		M_DWAController.setParameter(CPioneer3DXInterface::getInstance()->getParameter().dMaxV,     /*0.4 0.7*/
			CPioneer3DXInterface::getInstance()->getParameter().dMaxW,     /*30*D2R*/
			0.8, /*Acc*/
			0.3,     /*dRobotRadius*/
			0.35,
			0.25,
			1.,     /*1.0 Coeff_clearance1*/
			6.,      /*6.0 Coeff_heading6*/
			0.5);   /*0.5 Coeff_speed*/  //0.5


		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ROBOTSTATE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::GOALSTATE, true);


		break;
	}



	case BASIC:
	{
		CSensorDataManager::getInstance()->setSensorUsingFlag(CSensorDataManager::PIONEER3DX, true);
		//CSensorDataManager::getInstance()->setSensorUsingFlag(CSensorDataManager::VELODYNE, true);
		CSensorDataManager::getInstance()->setSensorUsingFlag(CSensorDataManager::HOKUYO, true);


		//M_RobotState.setPose(42, 19, 0, 0, 0, 0);
		//M_StateState = M_SubgoalState = M_RobotState;//
		//CRenderingObjects::getInstance()->setRobotState(M_RobotState);
		//M_GoalState.setPose(52, 18.3, 0, 0, 0, 0);


		M_StateState = M_SubgoalState = M_RobotState = CRenderingObjects::getInstance()->getRobotState();
		M_GoalState = CRenderingObjects::getInstance()->getGoalState();



		//M_SensorState.setX(0.); M_SensorState.setZ(0.24);
		M_SensorState.setX(0.2); M_SensorState.setZ(0.24);
		M_pRPFLocalizer = new CRangeBasedPFLocalizer;
		M_pRPFLocalizer->setParameter(1000, /*100 3000, */ //nMinParticleNum
			10000, /*1000 30000,*/  //nMaxParticleNum
			1000, /*100 3000 */ //nParticleDensity
			1.0, /*1.0 0.7*/  //dDeviationforTrans
			0.3, /*0.3 0.2*/  //dDeviationforRot
			M_PI / 180.0*20.0 / 1000.0, /*0.2*/   //dDeviationforTransToRot
			100, /*0.5*/    //nMinParticleSD
			15.0, //dRangeMaxDist
			1  //nRangeInterval
		);
		M_pRPFLocalizer->setMap(CMapManager::getInstance()->get2DGridMapState());
		M_pRPFLocalizer->initParticles(M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getYaw(), 0.3, 5 * D2R);
		M_pEstimatedPathList = new list<CRobotState>;
		M_pPathList = new list<CRobotState>;
		M_pPathPlanner = new CGradientPathPlanner(CMapManager::getInstance()->get2DGridMapState(), 0.3);
		M_pPathPlanner->setStartPose(M_RobotState);
		M_pPathPlanner->setGoalPose(M_GoalState);
		M_pPathPlanner->generatePath();
		M_pPathList = M_pPathPlanner->getPathList();

		//M_pLRFData = new double[181];


		//CPioneer3DXInterface::getInstance()->setMaxVelocity(0.25, 80.0*D2R);			//for safety (at 20181117)
		//CPioneer3DXInterface::getInstance()->setMaxVelocity(0.25, 40.0*D2R);			//20181215

		//CPioneer3DXInterface::getInstance()->setMaxVelocity(0.5, 80.0*D2R );

		M_DWAOldController.setParameter(CPioneer3DXInterface::getInstance()->getParameter().dMaxV,     /*0.4 0.7*/
			CPioneer3DXInterface::getInstance()->getParameter().dMaxW,     /*30*D2R*/
			0.8, /*Acc*/
			0.3,     /*dRobotRadius*/
			0.35,
			0.25,
			1.0,     /*1.0 Coeff_clearance1*/ //5
			6.0,      /*6.0 Coeff_heading6*/  //2
			0.5);   /*0.5 Coeff_speed*/  //2        0.5

		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::RANGEDATA0, true);
		//CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::POINTCLOUD2, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::PATHLIST0, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::GOALSTATE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ROBOTSTATE, true);
		//CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::SUBGOALSTATE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ESTIMATEDPATHLIST0, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::PARTICLES0, true);
		break;
	}

	case SIMULATION:
	{
		std::cout << "simulation" << endl;
		CSensorDataManager::getInstance()->setSensorUsingFlag(CSensorDataManager::PIONEER3DX, true);

		//スタート，ゴール位置指定ゾーン

		//ダブルクリック
		//M_StateState = M_SubgoalState = M_RobotState = CRenderingObjects::getInstance()->getRobotState();
		//M_GoalState = CRenderingObjects::getInstance()->getGoalState();

		//CKF_SFM SFM;
		//px, vx, py, vy
		//SFM.calcSequentPrediction(2., 0., 3., 0.);
		//SFM.calcSequentPrediction(2., 1., 3., 0.);


		
		//m_plrfvector_state
		//M_LRFNum = 361;
		//for (int i = 0; i < M_LRFNum; i++) {
		//	CRobotState state;
		//	state.setX(100.*cos((i*(180. / (M_LRFNum - 1)) - 90)*M_PI / 180.));
		//	state.setY(100.*sin((i*(180. / (M_LRFNum - 1)) - 90)*M_PI / 180.));
		//	M_pLRFVector_State.push_back(state);
		//	M_pLRFVector_r.push_back(100.);

		//}

		//07floorcut2 --- Clearance Check 20181231
		//M_RobotState.setX(42.);
		//M_RobotState.setY(4.6);
		//M_RobotState.setYaw(0.0*D2R);
		//M_StateState = M_SubgoalState = M_RobotState;
		//M_GoalState.setX(56.);
		//M_GoalState.setY(4.6);
		//M_GoalState.setYaw(0.0*D2R);
		//M_ObstacleState.setPose(48.3, 4.9, 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;


		////07floorcut --- toriaezu 20190105
		//M_RobotState.setX(4.);
		//M_RobotState.setY(4.);
		//M_RobotState.setYaw(0.0*D2R);
		//M_StateState = M_SubgoalState = M_RobotState;
		//M_GoalState.setX(10.);
		//M_GoalState.setY(4.);
		//M_GoalState.setYaw(0.0*D2R);
		//M_ObstacleState.setPose(5., 5., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;

		//これ使えば大丈夫(07floorcut.)
		//M_RobotState.setX(2.);
		//M_RobotState.setY(4.);
		//M_RobotState.setYaw(0.0*D2R);
		//M_StateState = M_SubgoalState = M_RobotState;
		//M_GoalState.setX(26.);
		//M_GoalState.setY(9.);
		//M_GoalState.setYaw(0.0*D2R);
		//M_ObstacleState.setPose(3., 3., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;

		//長いマップ(07floorcut2.)
		M_RobotState.setX(2.);
		M_RobotState.setY(8);
		M_RobotState.setYaw(-90.0*D2R);
		M_StateState = M_SubgoalState = M_RobotState;
		M_GoalState.setX(68.);
		M_GoalState.setY(9.5);
		//M_GoalState.setYaw(0.0*D2R);
		M_ObstacleState.setPose(3., 3., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		M_ObstacleStart = M_ObstacleState;

		//【逆走】長いマップ(07floorcut2.)
		M_RobotState.setX(68.);
		M_RobotState.setY(9.5);
		M_RobotState.setYaw(180.0*D2R);
		M_StateState = M_SubgoalState = M_RobotState;
		M_GoalState.setX(2.);
		M_GoalState.setY(8);
		//M_GoalState.setYaw(0.0*D2R);
		M_ObstacleState.setPose(3., 3., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		M_ObstacleStart = M_ObstacleState;

		//20201004 07floorcut.bmp tadokoro
		//M_RobotState.setX(4.);
		//M_RobotState.setY(4.);
		//M_RobotState.setYaw(0.0*D2R);
		//M_StateState = M_SubgoalState = M_RobotState;
		//M_GoalState.setX(25.);
		//M_GoalState.setY(9.);
		//M_GoalState.setYaw(0.0*D2R);
		//M_ObstacleState.setPose(3., 3., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;

		//m_statestate = m_subgoalstate = m_robotstate = crenderingobjects::getinstance()->getrobotstate();
		//m_goalstate = crenderingobjects::getinstance()->getgoalstate();

		//20201028 long corridor
		//M_RobotState.setX(40.);
		//M_RobotState.setY(5.);
		//M_RobotState.setYaw(0.0*D2R);
		//M_StateState = M_SubgoalState = M_RobotState;
		////M_GoalState.setX(15.);
		////M_GoalState.setY(4.);
		//M_GoalState.setX(57.);
		//M_GoalState.setY(6.);
		//M_GoalState.setYaw(0.0*D2R);

		//m_statestate = m_subgoalstate = m_robotstate = crenderingobjects::getinstance()->getrobotstate();
		//m_goalstate = crenderingobjects::getinstance()->getgoalstate();

		//M_StateState = M_SubgoalState = M_RobotState = CRenderingObjects::getInstance()->getRobotState();
		//M_GoalState = CRenderingObjects::getInstance()->getGoalState();

		M_pRPFLocalizer = new CRangeBasedPFLocalizer;
		M_pRPFLocalizer->setParameter(1000, /*100 3000, */ //nMinParticleNum
			10000, /*1000 30000,*/  //nMaxParticleNum
			1000, /*100 3000 */ //nParticleDensity
			1.0, /*1.0 0.7*/  //dDeviationforTrans
			0.3, /*0.3 0.2*/  //dDeviationforRot
			M_PI / 180.0*20.0 / 1000.0, /*0.2*/   //dDeviationforTransToRot
			100, /*0.5*/    //nMinParticleSD
			15.0, //dRangeMaxDist
			1  //nRangeInterval
		);
		M_pRPFLocalizer->setMap(CMapManager::getInstance()->get2DGridMapState());
		M_pRPFLocalizer->initParticles(M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getYaw(), 0.3, 5 * D2R);

		M_pRPFLocalizer_isao = new CRangeBasedPFLocalizer_isao;
		M_pRPFLocalizer_isao->setParameter(1000, /*100 3000, */ //nMinParticleNum
			10000, /*1000 30000,*/  //nMaxParticleNum
			1000, /*100 3000 */ //nParticleDensity
			1.0, /*1.0 0.7*/  //dDeviationforTrans
			0.3, /*0.3 0.2*/  //dDeviationforRot
			M_PI / 180.0*20.0 / 1000.0, /*0.2*/   //dDeviationforTransToRot
			100, /*0.5*/    //nMinParticleSD
			15.0, //dRangeMaxDist
			1  //nRangeInterval
		);
		M_pRPFLocalizer_isao->setMap(CMapManager::getInstance()->get2DGridMapState());
		M_pRPFLocalizer_isao->initParticles(M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getYaw(), 0.3, 5 * D2R);

		//CPioneer3DXInterface::getInstance()->setMaxVelocity(0.25, 40.0*D2R);

		M_pLRFData = new double[181];
		M_pEstimatedPathList = new list<CRobotState>;
		M_pEstimatedPathList1 = new list<CRobotState>;
		M_pEstimatedPathList2 = new list<CRobotState>;


		M_pPathPlanner = new CGradientPathPlanner(CMapManager::getInstance()->get2DGridMapState(), 0.3);
		M_pPathPlanner->setStartPose(M_RobotState);
		M_pPathPlanner->setGoalPose(M_GoalState);
		M_pPathPlanner->generatePath();
		M_pPathList = M_pPathPlanner->getPathList();

		//M_pRPFLocalizer = new CRangeBasedPFLocalizer;
		//M_pRPFLocalizer->setParameter(1000, /*100 3000, */ //nMinParticleNum
		//	10000, /*1000 30000,*/  //nMaxParticleNum
		//	1000, /*100 3000 */ //nParticleDensity
		//	1.0, /*1.0 0.7*/  //dDeviationforTrans
		//	0.3, /*0.3 0.2*/  //dDeviationforRot
		//	M_PI / 180.0*20.0 / 1000.0, /*0.2*/   //dDeviationforTransToRot
		//	100, /*0.5*/    //nMinParticleSD
		//	15.0, //dRangeMaxDist
		//	1  //nRangeInterval
		//);
		//M_pRPFLocalizer->setMap(CMapManager::getInstance()->get2DGridMapState());
		//M_pRPFLocalizer->initParticles(M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getYaw(), 0.3, 5 * D2R);

		M_RobotState_old = M_RobotState;


		//M_ObstacleState.setPose(20.0, 8.5, 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);

		//M_ObstacleState.setPose(12.0, 5., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;
		//M_ObstacleGoal.setPose(4., 3., 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);

		//M_ObstacleState.setPose(8.1, 4.2, 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);
		//M_ObstacleStart = M_ObstacleState;
		//M_ObstacleGoal.setPose(8., 4.2, 0.0, 0.0*D2R, 0.0*D2R, 0.0*D2R);

		M_DWAController.setParameter(CPioneer3DXInterface::getInstance()->getParameter().dMaxV,     /*0.4 0.7*/
			CPioneer3DXInterface::getInstance()->getParameter().dMaxW,     /*30*D2R*/
			0.8, /*Acc*/
			0.3,     /*dRobotRadius*/
			0.35,
			0.25,
			5.,     /*1.0 Coeff_clearance1*/
			2.,      /*6.0 Coeff_heading6*/
			2.);   /*0.5 Coeff_speed*/  //0.5

		M_pPointCloud_show0 = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ROBOTSTATE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::PATHLIST0, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::SUBGOALSTATE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::RANGEDATA0, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::DYNAMICOBSTACLE, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ESTIMATEDPATHLIST0, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ESTIMATEDPATHLIST1, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::ESTIMATEDPATHLIST2, true);
		CRenderingObjects::getInstance()->setRenderingFlag(CRenderingObjects::POINTCLOUD0, true);
		break;
	}

	case PRACTICE:
	{
		std::cout << "Init" << endl;



		cv::namedWindow("RGB image");
		break;
	}

	}
}

void CNavigationProcess::doProcess()
{
	const double DISTTOSUBGOAL = 2.0;
	const double DETERMINEGOAL = 0.5;

	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	if (!CSensorDataManager::getInstance()->readSensorData()) return;

	M_EncoderData = CSensorDataManager::getInstance()->getPioneer3DXEncoderData();
	M_pRangeData = CSensorDataManager::getInstance()->getHokuyoPointCloud();

	//Select subgoal ---------------------------------------------------------------------------------------------------------------------
	M_SubgoalState = this->selectSubgoal(M_RobotState, M_pPathList, DISTTOSUBGOAL);
	//M_SubgoalState = M_GoalState;

	M_Transform = Eigen::Affine3f::Identity();
	M_Transform.translation() << M_SensorState.getX(), M_SensorState.getY(), M_SensorState.getZ();
	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getRoll(), Eigen::Vector3f::UnitX()));
	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getPitch(), Eigen::Vector3f::UnitY()));
	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getYaw(), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*M_pRangeData, *M_pRangeData, M_Transform);
	M_VRangeData.clear();
	M_VRangeData = CTypeConverter::getInstance()->PCL2STLVector(M_pRangeData);

	//Localization --------------------------------------------------------------------------------------------------------------------------
	M_pRPFLocalizer->setControlInput(M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);
	M_pRPFLocalizer->setRangeData(M_VRangeData);
	M_pRPFLocalizer->estimateState();
	M_RobotState = M_pRPFLocalizer->getState();
	if (CSensorDataManager::getInstance()->getLoopCnt() % 5 == 0) M_pEstimatedPathList->push_back(M_RobotState);
	//--------------------------------------------------------------------------------------------------------------------------------------------
	//M_RobotState = CRenderingObjects::getInstance()->getRobotState();
	//M_GoalState.setPose(52, 18.3, 0, 0, 0, 0.0);
	//cout << M_GoalState.getX() << " " << M_GoalState.getY() << endl;
	//Motion control------------------------------------------------------------------------------------------------------------------------------
	M_KanayamaController.calculateControlVelocity(M_RobotState, M_SubgoalState);

	cout << "M_VRangeData Size : " << M_VRangeData.size() << " " << getPeriod() << endl;
	//CPioneer3DXInterface::getInstance()->ControlAgent(M_KanayamaController.getV(), M_KanayamaController.getW());
	//M_DWAController.calculateControlVelocity(M_RobotState, M_SubgoalState, M_pLRFData, getPeriod());
	M_DWAOldController.calculateControlVelocity(M_RobotState, M_SubgoalState, M_VRangeData, getPeriod());
	CPioneer3DXInterface::getInstance()->ControlAgent(M_DWAOldController.getV(), M_DWAOldController.getW());
	cout << "K : " << M_KanayamaController.getV() << " " << M_KanayamaController.getW()*R2D << endl;
	cout << "D : " << M_DWAOldController.getV() << " " << M_DWAOldController.getW()*R2D << endl;
	if (CCalculus::getInstance()->calculateDistance(M_RobotState.getPosition(), M_GoalState.getPosition()) < DETERMINEGOAL)
	{
		CPioneer3DXInterface::getInstance()->stop();
		// 		M_GoalState = M_StateState;
		// 		M_StateState = M_RobotState;
		// 		M_pPathPlanner->setRobotAndGoalState(M_StateState, M_GoalState);
		// 		M_pPathPlanner->generatePath();
		// 		M_pPathList = M_pPathPlanner->getPathList();
		this->terminate();
	}
	//--------------------------------------------------------------------------------------------------------------------------------------------

	M_Transform = Eigen::Affine3f::Identity();
	M_Transform.translation() << M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getZ();
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getRoll(), Eigen::Vector3f::UnitX()));
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getPitch(), Eigen::Vector3f::UnitY()));
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getYaw(), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*M_pRangeData, *M_pRangeData, M_Transform);

	CRenderingObjects::getInstance()->setPathList0(*M_pPathList, 0, 103, 0);
	CRenderingObjects::getInstance()->setGoalState(M_GoalState);
	CRenderingObjects::getInstance()->setSubgoalState(M_SubgoalState);
	CRenderingObjects::getInstance()->setRobotState(M_RobotState);
	CRenderingObjects::getInstance()->setEstimatedPathList0(M_pEstimatedPathList, 153, 0, 255);
	CRenderingObjects::getInstance()->setRangeData0(M_pRangeData);
	CRenderingObjects::getInstance()->setParticles0(M_pRPFLocalizer->getParticles(), M_pRPFLocalizer->getParticlesNum(), 255, 51, 0);
}

//void CNavigationProcess::doProcess()
//{
//	std::cout << endl;
//	const double DISTTOSUBGOAL = 1.0;
//	const double DETERMINEGOAL = 0.5;
//	//
//	CTimeString ts;
//	ProcessTime pt;
//
//	//static CKeyToggle key(VK_ADD, VK_ESCAPE);
//	//static int count_key = 0;
//	//if (key.isToggleChanged()) {
//	//	cout << "toggled!!" << endl;
//	//	count_key++;
//	//}
//	//cout << "count_key = " << count_key << endl;
//
//
//	//
//	static int num_frame = 1;
//	//pt.num_processFrame = num_frame;
//	//cout << "num_frame = " << pt.num_processFrame << endl;
//	cout << "num_frame = " << num_frame << endl;
//	num_frame++;
//
//	//
//	//pt.doProcess = ts.getTimeString();
//	//cout << "doProcess started     at " << pt.doProcess << endl;
//	//cout << "doProcess started     at " << ts.getTimeString() << endl;
//
//	CSensorDataManager::getInstance()->setDuration(this->getDuration());
//	if (!CSensorDataManager::getInstance()->readSensorData()) return;
//
//	M_EncoderData = CSensorDataManager::getInstance()->getPioneer3DXEncoderData();
//	M_pVelodyneRangeData = CSensorDataManager::getInstance()->getVelodyne2DPointCloud();
//
//	//
//	//pt.sensorGot = ts.getTimeString();
//	//cout << "sensor data got       at " << pt.sensorGot << endl;
//	//cout << "sensor data got       at " << ts.getTimeString() << endl;
//
//	//pcl::PointXYZI XXX;
//	//XXX.x = 10.0;
//	//XXX.y = 0.0;
//	//XXX.z = 0.0;
//	//XXX.intensity = 0.0;
//	//for (int i = 0; i < 1000; i++)
//	//{
//	//	M_pVelodyneRangeData->push_back(XXX);
//	//}
//
//	//Select subgoal ---------------------------------------------------------------------------------------------------------------------
//	//M_SubgoalState = this->selectSubgoal(M_RobotState, M_pPathList, DISTTOSUBGOAL);
//	M_SubgoalState = M_GoalState;
//
//	M_Transform = Eigen::Affine3f::Identity();
//	M_Transform.translation() << M_SensorState.getX(), M_SensorState.getY(), M_SensorState.getZ();
//	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getRoll(), Eigen::Vector3f::UnitX()));
//	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getPitch(), Eigen::Vector3f::UnitY()));
//	M_Transform.rotate(Eigen::AngleAxisf(M_SensorState.getYaw(), Eigen::Vector3f::UnitZ()));
//	pcl::transformPointCloud(*M_pVelodyneRangeData, *M_pVelodyneRangeData, M_Transform);
//
//	//convert PC number to 181
//	pcl::PointCloud<pcl::PointXYZI>::Ptr M_pVelodyneRangeData_361;
//	pcl::PointCloud<pcl::PointXYZI>::Ptr M_pVelodyneRangeData_181;
//	//M_pVelodyneRangeData_181 = CVelodyneInterface::getInstance()->convertPCto181PC(M_pVelodyneRangeData);
//	M_pVelodyneRangeData_361 = CVelodyneInterface::getInstance()->convertPCtoFewPC(M_pVelodyneRangeData, 361);
//
//
//	//
//	//pt.localization = ts.getTimeString();
//	//cout << "Localization started  at " << pt.localization << endl;
//	//cout << "Localization started  at " << ts.getTimeString() << endl;
//
//	//Localization --------------------------------------------------------------------------------------------------------------------------
//	M_pRPFLocalizer->setControlInput(M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);
//	//M_pRPFLocalizer->setRangeData(CTypeConverter::getInstance()->PCL2STLVector(M_pVelodyneRangeData));
//	M_pRPFLocalizer->setRangeData(CTypeConverter::getInstance()->PCL2STLVector(M_pVelodyneRangeData_361));
//	M_pRPFLocalizer->estimateState();
//	M_RobotState = M_pRPFLocalizer->getState();
//	if (CSensorDataManager::getInstance()->getLoopCnt() % 5 == 0) M_pEstimatedPathList->push_back(M_RobotState);
//	//--------------------------------------------------------------------------------------------------------------------------------------------
//
//	//Motion control------------------------------------------------------------------------------------------------------------------------------
//
//	//M_RobotState.setPose(10, 0, 0, 0, 0, 0);
//	//M_SubgoalState.setPose(15, 0, 0, 0, 0, 0);
//
//	//KanayamaController
//	//M_KanayamaController.calculateControlVelocity(M_RobotState, M_SubgoalState);
//	//CPioneer3DXInterface::getInstance()->ControlAgent(M_KanayamaController.getV(), M_KanayamaController.getW());
//	//cout << "V = " << M_KanayamaController.getV() << "  W = " << M_KanayamaController.getW() << endl;
//
//	//
//	//pt.control = ts.getTimeString();
//	//cout << "MotionControl started at " << pt.control << endl;
//	//cout << "MotionControl started at " << ts.getTimeString() << endl;
//
//	M_pVelodyneRangeData_181 = CVelodyneInterface::getInstance()->convertPCtoFewPC(M_pVelodyneRangeData_361, 181);
//	//M_pLRFData = CVelodyneInterface::getInstance()->convertPCto181DegArray(M_pVelodyneRangeData);		//can it copy deeply?
//	M_pLRFData = CVelodyneInterface::getInstance()->convert181PCto181Array(M_pVelodyneRangeData_181);
//	M_DWAController.calculateControlVelocity(M_RobotState, M_SubgoalState, M_pLRFData, getPeriod());
//
//	//debug
//	//cout << "M_EncoderData.dDeltaYaw[deg] = " << M_EncoderData.dDeltaYaw*180./M_PI << endl;
//
//	CPioneer3DXInterface::getInstance()->ControlAgent(M_DWAController.getV(), M_DWAController.getW());
//	//CPioneer3DXInterface::getInstance()->ControlAgent(M_DWAController.getV(), 0.);
//
//	std::cout << "V = " << M_DWAController.getV() << "  W = " << M_DWAController.getW() << endl;
//
//	std::cout << "X = " << M_RobotState.getX() << "  Y = " << M_RobotState.getY() << "  Yaw = " << M_RobotState.getYaw() << endl;
//
//
//	//DWAcheck
//	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_forDWAcheck = convertArraytoPCPtr(convertPCto181DegArray(M_pVelodyneRangeData),181);
//
//
//	if (CCalculus::getInstance()->calculateDistance(M_RobotState.getPosition(), M_GoalState.getPosition()) < DETERMINEGOAL)
//	{
//		CPioneer3DXInterface::getInstance()->stop();
//		// 		M_GoalState = M_StateState;
//		// 		M_StateState = M_RobotState;
//		// 		M_pPathPlanner->setRobotAndGoalState(M_StateState, M_GoalState);
//		// 		M_pPathPlanner->generatePath();
//		// 		M_pPathList = M_pPathPlanner->getPathList();
//		this->terminate();
//	}
//	//--------------------------------------------------------------------------------------------------------------------------------------------
//
//	short key_num = GetAsyncKeyState(VK_ADD);
//	static int count_toggle = 0;
//	if ((key_num & 1) == 1) {
//		std::cout << "toggled!!" << endl;
//			CPioneer3DXInterface::getInstance()->stop();
//			//string filename = "\../OutputText/output" + ts.getTimeString() + ".txt";
//
//			//ofstream outputfile(filename);
//			//outputfile << "start\n";
//
//			//for (auto itr = timelist.begin(); itr != timelist.end(); itr++) {
//			//	string s1f_output;
//			//	s1f_output += to_string(itr->num_processFrame) + "\n";
//			//	s1f_output += itr->doProcess + "\n";
//			//	s1f_output += itr->sensorGot + "\n";
//			//	s1f_output += itr->localization + "\n";
//			//	s1f_output += itr->control + "\n";
//			//	s1f_output += itr->rendering + "\n";
//			//	s1f_output += "\n";
//			//	outputfile << s1f_output;
//			//}
//			//outputfile << "end";
//			//outputfile.close();
//			//cout << "file saved" << endl;
//			//timelist.clear();
//			this->terminate();
//
//	}
//
//	M_Transform = Eigen::Affine3f::Identity();
//	M_Transform.translation() << M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getZ();
//	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getRoll(), Eigen::Vector3f::UnitX()));
//	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getPitch(), Eigen::Vector3f::UnitY()));
//	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getYaw(), Eigen::Vector3f::UnitZ()));
//
//	pcl::transformPointCloud(*M_pVelodyneRangeData_361, *M_pVelodyneRangeData_361, M_Transform);
//	//pcl::transformPointCloud(*M_pVelodyneRangeData, *M_pVelodyneRangeData, M_Transform);
//
//	//DWAcheck
//	//pcl::transformPointCloud(*cloud_forDWAcheck, *cloud_forDWAcheck, M_Transform);
//
//	//
//	//pt.rendering = ts.getTimeString();
//	//cout << "Rendering started     at " << pt.rendering << endl;
//	//cout << "Rendering started     at " << ts.getTimeString() << endl;
//
//
//	//CRenderingObjects::getInstance()->setPathList0(*M_pPathList, 0, 103, 0);
//	//CRenderingObjects::getInstance()->setSubgoalState(M_SubgoalState);
//	CRenderingObjects::getInstance()->setRobotState(M_RobotState);
//	CRenderingObjects::getInstance()->setEstimatedPathList0(M_pEstimatedPathList, 153, 0, 255);
//	CRenderingObjects::getInstance()->setPointCloud2(M_pVelodyneRangeData_361);
//	//CRenderingObjects::getInstance()->setPointCloud2(M_pVelodyneRangeData);
//
//	CRenderingObjects::getInstance()->setParticles0(M_pRPFLocalizer->getParticles(), M_pRPFLocalizer->getParticlesNum(), 255, 51, 0);
//
//	//CRenderingObjects::getInstance()->setPointCloud2(cloud_forDWAcheck);
//
//
//	std::cout << endl;
//
//	//
//	//timelist.push_back(pt);
//
//}

void CNavigationProcess::doProcess_Simulation()
{

	const double DISTTOSUBGOAL = 1.0;
	const double DETERMINEGOAL = 0.5;


	//std::cout << "process simulation" << endl;
	//↑必要

	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	if (!CSensorDataManager::getInstance()->readSensorData()) return;
	M_nLoopCnt = CSensorDataManager::getInstance()->getLoopCnt();

	// Define obstacle movement
	//moveObstacleState_polygon();


	M_pRangeData->clear();
	// Get sensor data
	//センサーデータを得ているゾーン
	getSimulationLRFRangeData(M_RobotState, M_ObstacleState, CMapManager::getInstance()->get2DGridMapState());

	// Localization
	//エンコーダデータを計算しているゾーン
	//M_RobotState     t
	//M_RobotState_old t-1
	M_EncoderData.dDeltaX = (M_RobotState.getX() - M_RobotState_old.getX())*cos(M_RobotState_old.getYaw()) + (M_RobotState.getY() - M_RobotState_old.getY())*sin(M_RobotState_old.getYaw());
	//M_EncoderData.dDeltaY = (M_RobotState_old.getX() - M_RobotState.getX())*sin(M_RobotState_old.getYaw()) + (M_RobotState.getY() - M_RobotState_old.getY())*cos(M_RobotState_old.getYaw());
	M_EncoderData.dDeltaY = -(M_RobotState.getX() - M_RobotState_old.getX())*sin(M_RobotState_old.getYaw()) + (M_RobotState.getY() - M_RobotState_old.getY())*cos(M_RobotState_old.getYaw());

	M_EncoderData.dDeltaYaw = M_RobotState.getYaw() - M_RobotState_old.getYaw();

	M_VRangeData.clear();
	M_VRangeData = CTypeConverter::getInstance()->PCL2STLVector(M_pRangeData);
	
	// Coordinate transform of range data
	M_Transform = Eigen::Affine3f::Identity();
	M_Transform.translation() << M_RobotState.getX(), M_RobotState.getY(), M_RobotState.getZ();
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getRoll(), Eigen::Vector3f::UnitX()));
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getPitch(), Eigen::Vector3f::UnitY()));
	M_Transform.rotate(Eigen::AngleAxisf(M_RobotState.getYaw(), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*M_pRangeData, *M_pRangeData, M_Transform);

	vector<int> i_feature_vec;
	for (int i = 0; i < M_pRangeData->size(); i++)
	{
		i_feature_vec.push_back((int)M_pRPFLocalizer_isao->getFeatureValue(M_pRangeData->points[i].x, M_pRangeData->points[i].y));
	}

	//preparing point cloud
	M_pPointCloud_show0->clear();
	for (int i = 0; i < M_pRangeData->size(); i++)
	{
		pcl::PointXYZRGB point_;
		point_.x = M_pRangeData->points[i].x;
		point_.y = M_pRangeData->points[i].y;
		point_.z = M_pRangeData->points[i].z;
		if (i_feature_vec[i] == 0)
		{
			point_.r = 255;
			point_.g = 255;
			point_.b = 255;
		}
		else
		{
			point_.r = 255;
			point_.g = 0;
			point_.b = 0;
		}
		M_pPointCloud_show0->push_back(point_);
	}

	{//ノイズを加えてるゾーン　getRandomValue(a):a*(-1～1)
		//double xNoise, tNoise;
		//xNoise = CCalculus::getInstance()->getRandomValue(M_EncoderData.dDeltaX * 0.10) - 0.05 * M_EncoderData.dDeltaX;
		//tNoise = CCalculus::getInstance()->getRandomValue(M_EncoderData.dDeltaYaw * 0.10);
		//M_EncoderData.dDeltaX += xNoise;
		//M_EncoderData.dDeltaYaw += tNoise;

		double xNoise, tNoise, yNoise;
		xNoise = CCalculus::getInstance()->getRandomValue(M_EncoderData.dDeltaX * 0.10) - 0.05 * M_EncoderData.dDeltaX;
		yNoise = CCalculus::getInstance()->getRandomValue(M_EncoderData.dDeltaY * 0.10);
		tNoise = CCalculus::getInstance()->getRandomValue(M_EncoderData.dDeltaYaw * 0.10);
		M_EncoderData.dDeltaX += xNoise;
		M_EncoderData.dDeltaY += yNoise;
		M_EncoderData.dDeltaYaw += tNoise;


	}

	M_pRPFLocalizer->setControlInput(M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);
	M_pRPFLocalizer->setRangeData(M_VRangeData);
	M_pRPFLocalizer->estimateState();
	M_PFRobotState = M_pRPFLocalizer->getState();


	M_pRPFLocalizer_isao->setControlInput(M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);
	M_pRPFLocalizer_isao->setRangeData(M_VRangeData);
	M_pRPFLocalizer_isao->setFeatureData(i_feature_vec);
	M_pRPFLocalizer_isao->estimateState();
	M_PFRobotState2 = M_pRPFLocalizer_isao->getState();


	//if (CSensorDataManager::getInstance()->getLoopCnt() % 5 == 0) M_pEstimatedPathList->push_back(M_RobotState);
	//M_RobotState = M_RobotState;
	M_pEstimatedPathList->push_back(M_RobotState);
	//cout << "M_RobotState.getX():" << M_RobotState.getX() << endl;
	//cout << "M_RobotState.getY():" << M_RobotState.getY() << endl;
	//↑t-1のロボット位置を出力してるけど要らないよね？

	M_pEstimatedPathList1->push_back(M_PFRobotState);
	M_pEstimatedPathList2->push_back(M_PFRobotState2);

	//Subgoal generation
	M_SubgoalState = this->selectSubgoal(M_RobotState, M_pPathList, DISTTOSUBGOAL);
	//M_SubgoalState = M_GoalState;

	M_RobotState_old = M_RobotState;

	// Control robot
	//KanayamaController
	//M_KanayamaController.calculateControlVelocity(M_RobotState, M_SubgoalState);
	//controlRobot(M_KanayamaController.getV(), M_KanayamaController.getW(), getPeriod());
	//cout << "V = " << M_KanayamaController.getV() << "  W = " << M_KanayamaController.getW() << endl;

	//DWA　移動ロボットの制御のコード
	M_DWAController.calculateControlVelocity(M_RobotState, M_SubgoalState, M_pLRFData, getPeriod());
	controlRobot(M_DWAController.getV(), M_DWAController.getW(), getPeriod());
	std::cout << "V = " << M_DWAController.getV() << "  W = " << M_DWAController.getW() << endl;

	std::cout << "X = " << M_RobotState.getX() << "  Y = " << M_RobotState.getY() << "  Yaw = " << M_RobotState.getYaw() << endl;

	std::cout << "PF_X = " << M_PFRobotState.getX() << "  PF_Y = " << M_PFRobotState.getY() << "  PF_Yaw = " << M_PFRobotState.getYaw() << endl << endl;
	//↑必要
	//std::cout << "" << M_PFRobotState.getX()<< endl;


	if (CCalculus::getInstance()->calculateDistance(M_RobotState.getPosition(), M_GoalState.getPosition()) < DETERMINEGOAL)
	{
		CPioneer3DXInterface::getInstance()->stop();
		// 		M_GoalState = M_StateState;
		// 		M_StateState = M_RobotState;
		// 		M_pPathPlanner->setRobotAndGoalState(M_StateState, M_GoalState);
		// 		M_pPathPlanner->generatePath();
		// 		M_pPathList = M_pPathPlanner->getPathList();

		//cout << "X" << endl;
		//for (auto itr = M_pEstimatedPathList->begin(); itr != M_pEstimatedPathList->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getX() << endl;
		//}
		//cout << "Y" << endl;
		//for (auto itr = M_pEstimatedPathList->begin(); itr != M_pEstimatedPathList->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getY() << endl;
		//}
		//cout << "Yaw" << endl;
		//for (auto itr = M_pEstimatedPathList->begin(); itr != M_pEstimatedPathList->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getYaw() << endl;
		//}

		//cout << "X_PF" << endl;
		//for (auto itr = M_pEstimatedPathList1->begin(); itr != M_pEstimatedPathList1->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getX() << endl;
		//}
		//cout << "Y_PF" << endl;
		//for (auto itr = M_pEstimatedPathList1->begin(); itr != M_pEstimatedPathList1->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getY() << endl;
		//}
		//cout << "Yaw_PF" << endl;
		//for (auto itr = M_pEstimatedPathList1->begin(); itr != M_pEstimatedPathList1->end(); itr++)
		//{
		//	CRobotState state = *itr;
		//	cout << state.getYaw() << endl;
		//}

		vector<vector<string>> s_output_vecvec;

		{
			vector<string> s_output_vec;
			s_output_vec.push_back("フレーム");
			s_output_vec.push_back("X");
			s_output_vec.push_back("Y");
			s_output_vec.push_back("Yaw");
			s_output_vec.push_back("X_PF");
			s_output_vec.push_back("Y_PF");
			s_output_vec.push_back("Yaw_PF");
			s_output_vec.push_back("X_PF_isao");
			s_output_vec.push_back("Y_PF_isao");
			s_output_vec.push_back("Yaw_PF_isao");
			s_output_vec.push_back("X_PF_delta");
			s_output_vec.push_back("Y_PF_delta");
			s_output_vec.push_back("Yaw_PF_delta");
			s_output_vec.push_back("X_PF_isao_delta");
			s_output_vec.push_back("Y_PF_isao_delta");
			s_output_vec.push_back("Yaw_PF_isao_delta");
			s_output_vec.push_back("Euclid_PF");
			s_output_vec.push_back("Euclid_PF_isao");
			s_output_vec.push_back("Euclid_delta");

			s_output_vecvec.push_back(s_output_vec);
		}

		{
			auto itr_PF = M_pEstimatedPathList1->begin();
			auto itr_PF_isao = M_pEstimatedPathList2->begin();
			int i_frame = 0;
			for (auto itr = M_pEstimatedPathList->begin(); itr != M_pEstimatedPathList->end(); itr++)
			{
				CRobotState State = *itr;
				CRobotState State_PF = *itr_PF;
				CRobotState State_PF_isao = *itr_PF_isao;
				double X_PF_delta = State_PF.getX() - State.getX();
				double Y_PF_delta = State_PF.getY() - State.getY();
				double Yaw_PF_delta = State_PF.getYaw() - State.getYaw();
				double euclid_PF = sqrt(pow(X_PF_delta, 2) + pow(Y_PF_delta, 2));
				double X_PF_isao_delta = State_PF_isao.getX() - State.getX();
				double Y_PF_isao_delta = State_PF_isao.getY() - State.getY();
				double Yaw_PF_isao_delta = State_PF_isao.getYaw() - State.getYaw();
				double euclid_PF_isao = sqrt(pow(X_PF_isao_delta, 2) + pow(Y_PF_isao_delta, 2));

				vector<string> s_output_vec;
				s_output_vec.push_back(to_string(i_frame));
				s_output_vec.push_back(to_string(State.getX()));
				s_output_vec.push_back(to_string(State.getY()));
				s_output_vec.push_back(to_string(State.getYaw()));
				s_output_vec.push_back(to_string(State_PF.getX()));
				s_output_vec.push_back(to_string(State_PF.getY()));
				s_output_vec.push_back(to_string(State_PF.getYaw()));
				s_output_vec.push_back(to_string(State_PF_isao.getX()));
				s_output_vec.push_back(to_string(State_PF_isao.getY()));
				s_output_vec.push_back(to_string(State_PF_isao.getYaw()));
				s_output_vec.push_back(to_string(X_PF_delta));
				s_output_vec.push_back(to_string(Y_PF_delta));
				s_output_vec.push_back(to_string(Yaw_PF_delta));
				s_output_vec.push_back(to_string(X_PF_isao_delta));
				s_output_vec.push_back(to_string(Y_PF_isao_delta));
				s_output_vec.push_back(to_string(Yaw_PF_isao_delta));
				s_output_vec.push_back(to_string(euclid_PF));
				s_output_vec.push_back(to_string(euclid_PF_isao));
				s_output_vec.push_back(to_string(euclid_PF_isao- euclid_PF));

				s_output_vecvec.push_back(s_output_vec);

				i_frame++;
				itr_PF++;
				itr_PF_isao++;
			}
		}
		string s_output_name = "E:/梅田研究室/07floorcut2/04シミュレーション18";
		//string s_output_name = "../JIROS2018/Task";
		s_output_name += "/" + CTimeString::getTimeString() + "_outputTrajectory.csv";
		CTimeString::getCSVFromVecVec(s_output_vecvec, s_output_name);

		this->terminate();

	}



	//M_Transform = Eigen::Affine3f::Identity();
	//M_Transform.translation() << M_PFRobotState.getX(), M_PFRobotState.getY(), M_PFRobotState.getZ();
	//M_Transform.rotate(Eigen::AngleAxisf(M_PFRobotState.getRoll(), Eigen::Vector3f::UnitX()));
	//M_Transform.rotate(Eigen::AngleAxisf(M_PFRobotState.getPitch(), Eigen::Vector3f::UnitY()));
	//M_Transform.rotate(Eigen::AngleAxisf(M_PFRobotState.getYaw(), Eigen::Vector3f::UnitZ()));
	//pcl::transformPointCloud(*M_pRangeData, *M_pRangeData, M_Transform);

	//  Visualization
	CRenderingObjects::getInstance()->setPathList0(*M_pPathList, 0, 255, 0);
	//↑スタートからゴールまでの道筋
	//CRenderingObjects::getInstance()->setPathList1(*M_pDyPathList, 255, 0, 0);

	CRenderingObjects::getInstance()->setSubgoalState(M_SubgoalState);
	CRenderingObjects::getInstance()->setRobotState(M_RobotState);
	//↑ロボット位置
	CRenderingObjects::getInstance()->setPFRobotState(M_PFRobotState);
	//↑PFのロボット位置
	//CRenderingObjects::getInstance()->setRangeData0(M_pRangeData->makeShared());
	CRenderingObjects::getInstance()->setDynamicObstacleState(M_ObstacleState);
	CRenderingObjects::getInstance()->setEstimatedPathList0(M_pEstimatedPathList, 0, 0, 255);
	//↑ロボット軌跡
	CRenderingObjects::getInstance()->setEstimatedPathList1(M_pEstimatedPathList1, 0, 0, 0);
	CRenderingObjects::getInstance()->setEstimatedPathList2(M_pEstimatedPathList2, 255, 0, 0);
	//↑PFのロボット軌跡
	//CRenderingObjects::getInstance()->setEstimatedPathList0(M_pEstimatedPathList1, 255, 0, 0);
	//CRenderingObjects::getInstance()->setParticles0(M_pRPFLocalizer->getParticles(), M_pRPFLocalizer->getParticlesNum(), 255, 51, 0);
	CRenderingObjects::getInstance()->setParticles0(M_pRPFLocalizer_isao->getParticles(), M_pRPFLocalizer_isao->getParticlesNum(), 255, 51, 0);
	CRenderingObjects::getInstance()->setPointCloud0(M_pPointCloud_show0);
}

void CNavigationProcess::doProcess_Test()
{
	const double DISTTOSUBGOAL = 1.0;
	const double DETERMINEGOAL = 0.5;

	std::cout << "process simulation" << endl;

	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	if (!CSensorDataManager::getInstance()->readSensorData()) return;
	M_nLoopCnt = CSensorDataManager::getInstance()->getLoopCnt();

	// Localization
	M_RobotState = M_RobotState;
	M_GoalState = CRenderingObjects::getInstance()->getGoalState();
	cout << "R" << M_RobotState.getX() << " " << M_RobotState.getY() << " " << M_RobotState.getYaw()*R2D << endl;
	cout << "G" << M_GoalState.getX() << " " << M_GoalState.getY() << " " << M_GoalState.getYaw()*R2D << endl;
	// Control robot

	//KanayamaController
	M_KanayamaController.calculateControlVelocity(M_RobotState, M_GoalState);
	cout << "Vk = " << M_KanayamaController.getV() << "  Wk = " << M_KanayamaController.getW() << endl;
	//CPioneer3DXInterface::getInstance()->ControlAgent(M_KanayamaController.getV(), M_KanayamaController.getW());


	////DWA
	M_DWAController.calculateControlVelocity(M_RobotState, M_GoalState, M_pLRFData, getPeriod());
	std::cout << "Vd = " << M_DWAController.getV() << "  Wd = " << M_DWAController.getW() << endl;
	CPioneer3DXInterface::getInstance()->ControlAgent(M_DWAController.getV(), M_DWAController.getW());


	//  Visualization
	CRenderingObjects::getInstance()->setGoalState(M_GoalState);
	CRenderingObjects::getInstance()->setRobotState(M_RobotState);
}

void CNavigationProcess::moveObstacleState_polygon() {

	double x, y;
	double speed;		//進むスピ?ド
	double angle_Avoid = M_PI * 1 / 3.;		//この角度は-xに?っすぐで0
	double x_AvoidStart = 10.;		//6は仮
	x = M_ObstacleState.getX();
	y = M_ObstacleState.getY();


	if (M_ObstacleGoal.getX() < x) {//走行の条件

		if (x <= x_AvoidStart && M_ObstacleGoal.getY() <= y) {//回避の条件
															  //speed = 0.7;
			speed = 1.;
			M_ObstacleState.setX(x + speed * cos(angle_Avoid + M_PI)*getPeriod()*0.001);
			M_ObstacleState.setY(y + speed * sin(angle_Avoid + M_PI)*getPeriod()*0.001);

		}
		else {//直進のみの条件
			speed = 1.;
			M_ObstacleState.setX(x + speed * cos(0 + M_PI)*getPeriod()*0.001);
			M_ObstacleState.setY(y + speed * sin(0 + M_PI)*getPeriod()*0.001);
		}
	}

}


void CNavigationProcess::moveObstacleState_cos() {

	double x, y;
	double speed;		//進むスピ?ド
	double angle_Avoid = M_PI * 1 / 3.;		//この角度は-xに?っすぐで0
	double x_AvoidStart = 10.;		//avoidance starts from (10,4)
	x = M_ObstacleState.getX();
	y = M_ObstacleState.getY();

	//もしすれ違いの角度が0ならpolygonの方を呼び出す．
	if (angle_Avoid == 0.) {
		moveObstacleState_polygon();
		return;
	}


	double Amplitude = (M_ObstacleStart.getY() - M_ObstacleGoal.getY()) / 2.;

	double lambda = 2 * (2 * Amplitude) / tan(angle_Avoid);

	double Wavenumber = 2 * M_PI / lambda;

	double theta0 = 0. - Wavenumber * x_AvoidStart;

	//この時?でcosの式が完成
	double x_delta, y_delta;

	if (M_ObstacleGoal.getX() < x) {//走行の条件

		if (x <= x_AvoidStart && x_AvoidStart - lambda / 2. <= x) {//回避の条件
																   //speed = 0.7;
			speed = 1.;

			x_delta = -(speed * getPeriod()*0.001) / sqrt(1 + pow(Amplitude*Wavenumber*sin(Wavenumber * x + theta0), 2.));
			y_delta = -Amplitude * Wavenumber*sin(Wavenumber*x + theta0) * x_delta;

			M_ObstacleState.setX(x + x_delta);
			M_ObstacleState.setY(y + y_delta);

		}
		else {//直進のみの条件
			speed = 1.;
			M_ObstacleState.setX(x + speed * cos(0 + M_PI)*getPeriod()*0.001);
			M_ObstacleState.setY(y + speed * sin(0 + M_PI)*getPeriod()*0.001);

		}
	}

}


void CNavigationProcess::getSimulationLRFRangeData(CRobotState RobotState, CRobotState M_ObstacleState, CTWODMapState * pMap)
{
	double x = RobotState.getX(); double y = RobotState.getY(); double t = RobotState.getYaw();
	double dRangeMaxDist = 50.0;

	int nPosX1 = 0, nPosY1 = 0;
	int nPosX2 = 0, nPosY2 = 0;
	int nHitX = 0, nHitY = 0;	// Ray-tracingﾀｻ ﾅ・?ｾｺ ﾁｵｿ｡ｼｭﾀﾇ ｻﾃ ｰ｢ｰ｢ﾀﾇ ｰﾅｸｮﾁ､ｺｸ

	//
	int obPosX, obPosY;			//obstacle's position in cell

	double dAngle = 0.;
	double dPosTh = 0.;

	double dCellSize = pMap->getCellSize();

	//
	CTWODMapState CopyMap(pMap->getMapSizeX(), pMap->getMapSizeY(), pMap->getCellSize());
	CopyMap.set2DGridMap(pMap->get2DGridMap());

	//
	obPosX = (int)(M_ObstacleState.getX() / dCellSize);
	obPosY = (int)(M_ObstacleState.getY() / dCellSize);
	//fill obstacle data into map (squared shape without corner)
	for (int j = -2; j <= 2; j++) {
		for (int i = -2; i <= 2; i++) {
			if (abs(i) == 2 && abs(j))continue;
			CopyMap.set2DGridMapElement(obPosX + i, obPosY + j, CTWODMapState::OCCUPIED);

		}
	}

	// 100ﾀｸｷﾎ ｳｪｴｲﾁﾖｴﾂ ﾀﾌﾀｯｴﾂ 10cmｰﾝﾀﾚｿ｡ ｸﾂﾃ郛ｭ ｹ貮簑ｻ ﾀﾌｵｿｽﾃﾅｰｱ・ﾀｧﾇﾔﾀﾌｴﾙ.
	nPosX1 = (int)(x / dCellSize);		// ｻﾃﾀﾇ ﾀｧﾄ｡ｿ｡ｼｭ ｷｹﾀﾌﾀ・ｼｾｼｭ ｹ貮篩｡ ｵ鄕･ Xｹ・Eｹ貮篌､ﾅﾍ
	nPosY1 = (int)(y / dCellSize);			// ｻﾃﾀﾇ ﾀｧﾄ｡ｿ｡ｼｭ ｷｹﾀﾌﾀ・ｼｾｼｭ ｹ貮篩｡ ｵ鄕･ Yｹ・Eｹ貮篌､ﾅﾍ

	pcl::PointXYZ RangePoint;
	//ｱ､ｼｱﾇﾏｳｪﾇﾏｳｪ
	double r;
	double angle;

	for (int i = -90; i <= 90; i = i + 1)
	{
		//angle = i * (180. / (M_LRFNum - 1)) - 90.;
		nPosX2 = nPosX1 + (int)(dRangeMaxDist*cos(t + i * D2R) / dCellSize);	// ｼｾｼｭ ｰｨﾁ・ｰﾅｸｮｿ｡ ｵ鄕･ Xｹ貮簑ﾇ ﾃﾖｴ・ｰｨﾁ・ｹ・?ｼｳﾁ､.	
		nPosY2 = nPosY1 + (int)(dRangeMaxDist*sin(t + i * D2R) / dCellSize);	// ｼｾｼｭ ｰｨﾁ・ｰﾅｸｮｿ｡ ｵ鄕･ Yｹ貮簑ﾇ ﾃﾖｴ・ｰｨﾁ・ｹ・?ｼｳﾁ､.

		doRayTracing(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY, &CopyMap); // Ray-tracing ｼ・E
																				// ﾁｵﾀﾇ ｰ豌・ｱﾙﾃｳｿ｡ ﾀﾖｴﾂ ｰ豼・?ｵ鈆ﾂ ｺﾎｺﾐ

		r = sqrt((double)(nHitX*dCellSize - nPosX1 * dCellSize)*(nHitX*dCellSize - nPosX1 * dCellSize) +
			(double)(nHitY*dCellSize - nPosY1 * dCellSize)*(nHitY*dCellSize - nPosY1 * dCellSize));

		//M_pLRFVector_r[i] = r;
		M_pLRFData[i + 90] = r;

		RangePoint.x = r * cos((i)*D2R);
		RangePoint.y = r * sin((i)*D2R);
		RangePoint.z = 1.0;
		M_pRangeData->push_back(RangePoint);
	}

	//for (int i =0; i<M_LRFNum; i = i + 1)
	//{
	//	angle = i*(180./(M_LRFNum - 1))-90.;
	//	nPosX2 = nPosX1 + (int)(dRangeMaxDist*cos(t+angle*D2R)/dCellSize);	// ｼｾｼｭ ｰｨﾁ・ｰﾅｸｮｿ｡ ｵ鄕･ Xｹ貮簑ﾇ ﾃﾖｴ・ｰｨﾁ・ｹ・?ｼｳﾁ､.	
	//	nPosY2 = nPosY1 + (int)(dRangeMaxDist*sin(t+angle*D2R)/dCellSize);	// ｼｾｼｭ ｰｨﾁ・ｰﾅｸｮｿ｡ ｵ鄕･ Yｹ貮簑ﾇ ﾃﾖｴ・ｰｨﾁ・ｹ・?ｼｳﾁ､.

	//	doRayTracing(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY, &CopyMap); // Ray-tracing ｼ・E
	//	// ﾁｵﾀﾇ ｰ豌・ｱﾙﾃｳｿ｡ ﾀﾖｴﾂ ｰ豼・?ｵ鈆ﾂ ｺﾎｺﾐ

	//	r = sqrt((double)(nHitX*dCellSize - nPosX1 * dCellSize)*(nHitX*dCellSize - nPosX1 * dCellSize) +
	//		(double)(nHitY*dCellSize - nPosY1 * dCellSize)*(nHitY*dCellSize - nPosY1 * dCellSize));

	//	RangePoint.x = r*cos( (angle)*D2R);
	//	RangePoint.y = r*sin( (angle)*D2R );
	//	RangePoint.z = 1.0;
	//	M_pRangeData->push_back(RangePoint);


	//	//M_pLRFVector_r[i] = r;
	//	//M_pLRFData[i] = r;		M_pLRFVector_State[i].setX(r*cos((angle)*D2R));
	//	M_pLRFVector_State[i].setX(r*cos((angle)*D2R));
	//	M_pLRFVector_State[i].setY(r*sin((angle)*D2R));

	//}

}

void CNavigationProcess::controlRobot(double v, double w, double dDt)
{
	double dt = dDt * 0.001;
	M_RobotState.setX(M_RobotState.getX() + dt * v*cos(M_RobotState.getYaw()));
	M_RobotState.setY(M_RobotState.getY() + dt * v*sin(M_RobotState.getYaw()));
	M_RobotState.setYaw(M_RobotState.getYaw() + w * dt);
}


void CNavigationProcess::doProcess_Practice()
{
	std::cout << "process" << endl;
	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	if (!CSensorDataManager::getInstance()->readSensorData()) return;



}


CRobotState CNavigationProcess::selectSubgoal(CRobotState RobotState, list<CRobotState> * PathList, double dDist)
{
	while (1)
	{
		if (PathList->size() <= 1) break;
		if (CCalculus::getInstance()->calculateDistance(RobotState.getPosition(), PathList->front().getPosition()) > dDist) break;
		PathList->pop_front();
	}
	return  PathList->front();
}


void CNavigationProcess::stateArray2stateList(CRobotState * Array, list<CRobotState> * pList)
{
	int n = 0;
	if (pList->size() > 0) pList->clear();
	while (!(Array[n].getX() == 0.0 && Array[n].getY() == 0.0) && n != 10)
	{
		pList->push_back(Array[n]);
		n++;
	}
}


/**
 @brief : Function executing ray-tracing with vector of samples' position
*/
void CNavigationProcess::doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y, CTWODMapState * pMap)
{

	int eps = 0;
	int x_thres = x_2, y_thres = y_2;

	if (x_1 < 0) x_1 = 0;
	else if (x_1 >= pMap->getMapSizeX()) x_1 = pMap->getMapSizeX() - 1;
	if (y_1 < 0) y_1 = 0;
	else if (y_1 >= pMap->getMapSizeY()) y_1 = pMap->getMapSizeY() - 1;

	//ﾀﾌｰﾍﾀｻ ﾁﾖｼｮ ﾇﾘﾁｦﾇﾏｸ・ﾀﾚｵｵ ｰ豌霄ﾎｱﾙｿ｡ｼｭ ｷｹﾀﾌﾆｮｷ｡ﾀﾌｽﾌｰ皺嵓｡ ﾀﾌｻﾘﾁ・
	// 	if(x_thres < 0) x_thres = 0;
	// 	else if(x_thres >= m_pMap->getMapSizeX()) x_thres = m_pMap->getMapSizeX()-1;
	// 	if(y_thres < 0) y_thres = 0;
	// 	else if(y_thres >= m_pMap->getMapSizeY()) y_thres = m_pMap->getMapSizeY()-1;

	int x = x_1, y = y_1;

	int delx = x_thres - x_1;
	int dely = y_thres - y_1;


	if (delx > 0) {
		if (dely >= 0) {
			if (delx > dely) {
				for (int x = x_1 + 2; x <= x_thres; x++) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if ((eps << 1) >= delx) {
						y++;
						eps -= delx;
					}
				}

			}
			else { //delx <= delyﾀﾎ ｰ豼・

				for (y = y_1 + 2; y <= y_thres; y++) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if ((eps << 1) >= dely) {
						x++;
						eps -= dely;
					}
				}

			}

		}
		else { // dely < 0ﾀﾎｰ貘ｯ
			if (delx > -dely) {
				for (x = x_1 + 2; x <= x_thres; x++) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if ((eps << 1) <= -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //delx <= -delyﾀﾎ ｰ豼・

				for (y = y_1 - 2; y >= y_thres; y--) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if ((eps << 1) >= -dely) {
						x++;
						eps += dely;
					}
				}

			}
		}
	}

	else { //delx <=0ﾀﾎｰ豼・
		if (dely >= 0) {
			if (-delx > dely) {

				for (x = x_1 - 2; x >= x_thres; x--) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if ((eps << 1) >= -delx) {
						y++;
						eps += delx;
					}
				}

			}
			else { //-delx <= delyﾀﾎｰ豼・

				for (y = y_1 + 2; y <= y_thres; y++) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if ((eps << 1) <= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
		else { //dely < 0ﾀﾎｰ豼・
			if (-delx > -dely) {
				for (x = x_1 - 2; x >= x_thres; x--) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= dely;
					if ((eps << 1) > -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //-delx <= -delyﾀﾎｰ豼・
				for (y = y_1 - 2; y >= y_thres; y--) {
					if (pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED) {
						// ｰﾅｸｮｰｪﾀｻ ｳﾖｴﾂｴﾙ.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= delx;
					if ((eps << 1) >= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
	}
}



void CNavigationProcess::execute(int nPeriod, int nProcMode)
{
	m_nProcMode = nProcMode;
	this->initialize();
	CProcessThread::start(Thread, this, nPeriod);
}

void CNavigationProcess::terminate()
{
	CProcessThread::terminate();
}