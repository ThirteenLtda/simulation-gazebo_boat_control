#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include "ModelPositionController.hpp"
#include <string>
#include <algorithm>

//
//  Verificando se na DLL do PLugin ocorreram os eventos de ATTACH e DETACH (Fechamento do Unity)
//
#ifdef WIN32
BOOL WINAPI DllMain(
	HINSTANCE hinstDLL,  // handle to DLL module
	DWORD fdwReason,     // reason for calling function
	LPVOID lpReserved)  // reserved
{
	// Perform actions based on the reason for calling.
	switch (fdwReason)
	{
            case DLL_PROCESS_ATTACH:
                break;
            case DLL_THREAD_ATTACH:
                // Do thread-specific initialization.
                break;

            case DLL_THREAD_DETACH:
                // Do thread-specific cleanup.
                break;
            case DLL_PROCESS_DETACH:
                break;
        }
	return TRUE;
}
#endif


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelPositionController)

ModelPositionController::ModelPositionController() : ModelPlugin()
{
    _useController = false;
}

ModelPositionController::~ModelPositionController()
{
}

//
//  Sample XML plugin tag
//
//	<plugin name = "ModelPositionController" filename = "libModelPositionController.dll">
//		<info filepath = "simulation.txt" timestep="0.25" interpolate="false"/>
//	</plugin>

//  <plugin name = "ModelPositionController" filename = "libModelPositionController">
//  <info filepath = "simulation.txt" timestep = "0.25" interpolate = "true" start_time="10"/ >
//  <controller type = "sliding" pk = "10000" pa = "2" rk = "5000" ra = "1" / >
//  < / plugin>


void ModelPositionController::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    gzmsg << "ModelPositionController::Load() : Model Name : " << model->GetName();
		
    _world = model->GetWorld();

    this->_model = model;

    #ifdef WIN32
        this->_updateConnection = _world->ConnectWorldUpdateBegin(boost::bind(&ModelPositionController::Update, this, _1));
    #else
        this->_updateConnection= event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPositionController::Update, this, _1));
    #endif

    this->_interpolatePose = true;
    this->_timeStep = 0.1f;

    if( sdf->HasElement("info"))
	{
        if (sdf->GetElement("info")->HasAttribute("filepath") == false)
		{
			gzthrow("ModelPositionController::Load() SDF Parameter[info->filepath] not found");
			return;
		}
		
        std::string filepath = sdf->GetElement("info")->GetAttribute("filepath")->GetAsString();
        gzmsg << "ModelPositionController::Load() SDF Parameter[info->filepath] : " << filepath;
		
        if (sdf->GetElement("info")->HasAttribute("timestep"))
		{
            std::string strTimeStep = sdf->GetElement("info")->GetAttribute("timestep")->GetAsString();
			std::stringstream ss(strTimeStep);
            ss >> _timeStep;
            gzmsg << "ModelPositionController::Load() timeStep : " << _timeStep;
		}

        if (sdf->GetElement("info")->HasAttribute("interpolate"))
		{
            std::string strBool = sdf->GetElement("info")->GetAttribute("interpolate")->GetAsString();

			if (strBool.compare("0")==0)
			{
                this->_interpolatePose = false;
				gzmsg << "ModelPositionController::Load() interpolate : false";
			}
			else
			{
				gzmsg << "ModelPositionController::Load() interpolate : true";
			}
		}

        if (sdf->GetElement("info")->HasAttribute("start_time"))
        {
            std::string strStartTime = sdf->GetElement("info")->GetAttribute("start_time")->GetAsString();
            std::stringstream ss(strStartTime);
            ss >> _startTime;
            gzmsg << "ModelPositionController::Load() startTime : " << _startTime;
        }
        else
        {
            _startTime = 0.0;
            gzmsg << "ModelPositionController::Load() no start_time defined. Setting start_time to zero.";
        }

        _poseList.clear();

		std::string line;
		std::ifstream myfile(filepath);

		if (!myfile) //Always test the file open.
		{
			gzmsg << "ModelPositionController::Load() ERROR File not found";
			return;
		}

        // Contruir lista com os poses da embaracação ao longo do tempo e também lista
        //com as acelerações lineares e angulares.
        // Os angulos do arquivo estão em graus e serão salvos na lista em radianos.
		while (std::getline(myfile, line))
		{
            double px, py, pz, rx, ry, rz; //Poses
            double apx, apy, apz, arx, ary, arz; //Acelerações

            sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &px, &py, &pz, &rx, &ry, &rz, &apx, &apy, &apz, &arx, &ary, &arz);

            //LOGGER.Write(Log::INFO, "		%lf %lf %lf %lf %lf %lf", px, py, pz, rx, ry, rz);

            PoseInfo *poseInfo = new PoseInfo();
            PoseInfo *accelerationInfo = new PoseInfo();

            poseInfo->pos_x = px;
            poseInfo->pos_y = py;
            poseInfo->pos_z = pz;

            accelerationInfo->pos_x = apx;
            accelerationInfo->pos_y = apy;
            accelerationInfo->pos_z = apz;

			math::Angle angleX, angleY, angleZ;

            angleX.SetFromDegree(rx);
			angleY.SetFromDegree(ry);
			angleZ.SetFromDegree(rz);

            poseInfo->rot_x = angleX.Radian();
            poseInfo->rot_y = angleY.Radian();
            poseInfo->rot_z = angleZ.Radian();

            math::Angle angularAccelX, angularAccelY, angularAccelZ;

            angularAccelX.SetFromDegree(arx);
            angularAccelY.SetFromDegree(ary);
            angularAccelZ.SetFromDegree(arz);

            accelerationInfo->rot_x = angularAccelX.Radian();
            accelerationInfo->rot_y = angularAccelY.Radian();
            accelerationInfo->rot_z = angularAccelZ.Radian();

            _poseList.push_back(poseInfo);

            _accelerationList.push_back(accelerationInfo);
		}
	}

    int count = _poseList.size();

    if (count == 0)
    {
        gzthrow("ModelPositionController::Load()  PoseList size == 0!");
        return;
    }
    else
        gzmsg << "ModelPositionController::Load()  Read " << count << " poses";


    //
    // Configuração dos valores dos controladores.
    //

    float pk = 10000.0f;
	float pa = 2.0f;
	float rk = 5000.0f;
	float ra = 1.0f;

    float pP = 10000.0f;
    float pD = 0.0f;
    float pI = 0.0f;
    float rP = 50000.0f;
    float rD = 0.0f;
    float rI = 0.0f;

    if (sdf->HasElement("controller"))
	{
		_useController = true;

        if (sdf->GetElement("controller")->HasAttribute("type") )
		{
            _controllerType = sdf->GetElement("controller")->GetAttribute("type")->GetAsString();
            gzmsg << "ModelPositionController::Load() controller : %s", _controllerType;

            if (_controllerType == "sliding")
            {
                gzmsg << "ModelPositionController::Load() controller type == sliding";
            }
            else if(_controllerType == "pid")
            {
                gzmsg << "ModelPositionController::Load() controller type == pid";
            }
            else
            {
                gzmsg << "ModelPositionController::Load() controller type (%s) not found. Controller can only be sliding or pid!!! Using pid..." << _controllerType;
                _controllerType != "pid";
            }
		}

        if (sdf->GetElement("controller")->HasAttribute("usePrecalculatedForces"))
        {
            std::string strBool = sdf->GetElement("controller")->GetAttribute("usePrecalculatedForces")->GetAsString();

            if (strBool.compare("0")==0)
            {
                this->_usePrecalculatedForces = false;
                gzmsg << "ModelPositionController::Load() usePrecalculatedForces : false";
            }
            else
            {
                this->_usePrecalculatedForces = true;
                gzmsg << "ModelPositionController::Load() usePrecalculatedForces : true";
            }
        }
        else
        {
            gzmsg << "ModelPositionController::Load() usePrecalculatedForces not found. Using true";
            this->_usePrecalculatedForces = true;
        }


        //SLIDING PARAMETERS
        if (sdf->GetElement("controller")->HasAttribute("pk"))
		{
            std::string strPK = sdf->GetElement("controller")->GetAttribute("pk")->GetAsString();
			std::stringstream ss(strPK);
			ss >> pk;
			gzmsg << "ModelPositionController::Load() Controller SlidingMode position k : " << pk;
		}

        if (sdf->GetElement("controller")->HasAttribute("pa"))
		{
            std::string strPA = sdf->GetElement("controller")->GetAttribute("pa")->GetAsString();
			std::stringstream ss(strPA);
			ss >> pa;
			gzmsg << "ModelPositionController::Load() Controller SlidingMode position a : " << pa;
		}

        if (sdf->GetElement("controller")->HasAttribute("rk"))
		{
            std::string strPK = sdf->GetElement("controller")->GetAttribute("rk")->GetAsString();
			std::stringstream ss(strPK);
			ss >> rk;
			gzmsg << "ModelPositionController::Load() Controller SlidingMode rotation k : " << rk;
		}

        if (sdf->GetElement("controller")->HasAttribute("ra"))
		{
            std::string strPA = sdf->GetElement("controller")->GetAttribute("ra")->GetAsString();
			std::stringstream ss(strPA);
			ss >> ra;
			gzmsg << "ModelPositionController::Load() Controller SlidingMode rotation a : " << ra;
		}

        //PID PARAMETERS
        if (sdf->GetElement("controller")->HasAttribute("pP"))
        {
            std::string strPP = sdf->GetElement("controller")->GetAttribute("pP")->GetAsString();
            std::stringstream ss(strPP);
            ss >> pP;
            gzmsg << "ModelPositionController::Load() Controller PID position P : " << pP;
        }

        if (sdf->GetElement("controller")->HasAttribute("pI"))
        {
            std::string strPI = sdf->GetElement("controller")->GetAttribute("pI")->GetAsString();
            std::stringstream ss(strPI);
            ss >> pI;
            gzmsg << "ModelPositionController::Load() Controller PID position I : " << pI;
        }

        if (sdf->GetElement("controller")->HasAttribute("pD"))
        {
            std::string strPD = sdf->GetElement("controller")->GetAttribute("pD")->GetAsString();
            std::stringstream ss(strPD);
            ss >> pD;
            gzmsg << "ModelPositionController::Load() Controller PID position D : " << pD;
        }

        if (sdf->GetElement("controller")->HasAttribute("rP"))
        {
            std::string strRP = sdf->GetElement("controller")->GetAttribute("rP")->GetAsString();
            std::stringstream ss(strRP);
            ss >> rP;
            gzmsg << "ModelPositionController::Load() Controller PID rotation P : " << rP;
        }

        if (sdf->GetElement("controller")->HasAttribute("rI"))
        {
            std::string strRI = sdf->GetElement("controller")->GetAttribute("rI")->GetAsString();
            std::stringstream ss(strRI);
            ss >> rI;
            gzmsg << "ModelPositionController::Load() Controller PID rotation I : " << rI;
        }

        if (sdf->GetElement("controller")->HasAttribute("rD"))
        {
            std::string strRD = sdf->GetElement("controller")->GetAttribute("rD")->GetAsString();
            std::stringstream ss(strRD);
            ss >> rD;
            gzmsg << "ModelPositionController::Load() Controller PID rotation D : " << rD;
        }
	}

    _registerFirstLoop = false;

	if (_useController)
	{	
        if (_controllerType == "sliding")
        {
            for (int i = 0; i < 3; i++)
            {
                _controlledAxisArray[i].Init(pk, pa);
            }

            for (int i = 3; i < 6; i++)
            {
                _controlledAxisArray[i].Init(rk, ra);
            }
        }
        else
        {

            for (int i = 0; i < 3; i++)
            {
                _controlledAxisArray[i].Init(pP, pI , pD);
            }

            for (int i = 3; i < 6; i++)
            {
                _controlledAxisArray[i].Init(rP, rI , rD);
            }
        }
	}

    if (_startTime < 0.0 || _startTime > (double) (_poseList.size() - 1) * _timeStep )
    {
        gzmsg << "ModelPositionController::Load()  Invalid start_time. Using start_time = 0.";
    }

    _currentPoseIndex = (int) (_startTime / _timeStep);
    _timeAcum = _startTime - _currentPoseIndex * _timeStep;

    //Trazer embarcação para posição inicial
    SetModelPosition(_currentPoseIndex);

    gzmsg << "ModelPositionController::Load() : Model Name : " << model->GetName();
}

void ModelPositionController::Reset()
{
	gzmsg << "ModelPositionController::Reset()";

    _currentPoseIndex = (int) (_startTime / _timeStep);
    _timeAcum = _startTime - _currentPoseIndex * _timeStep;

    _registerFirstLoop = false;

    ResetModelToStartPose();
}

void ModelPositionController::SetModelPosition(int index)
{
    gazebo::math::Pose pose;
    pose.pos = gazebo::math::Vector3(_poseList[index]->pos_x,_poseList[index]->pos_y,_poseList[index]->pos_z);

    math::Angle angleX, angleY, angleZ;
    angleX.SetFromRadian(_poseList[index]->rot_x);
    angleY.SetFromRadian(_poseList[index]->rot_y);
    angleZ.SetFromRadian(_poseList[index]->rot_z);

    pose.rot.SetFromEuler(angleX.Radian(), angleY.Radian(), angleZ.Radian());

    this->_model->SetWorldPose(pose);
}

void ModelPositionController::ResetModelToStartPose()
{
    int _startPoseIndex = (int) (_startTime / _timeStep);
    SetModelPosition(_startPoseIndex);
}

void ModelPositionController::CheckSimulationReachedEnd()
{
    if (_currentPoseIndex >= _poseList.size()) // Chegou ao fim da simulação.
    {
        gzmsg << "ModelPositionController::CheckSimulationReachedEnd()";
        _world->Reset(); //RESETA MUNDO!!!!!!!!!!!!!!!
    }
}

void ModelPositionController::Update(const common::UpdateInfo &info)
{
	//LOGGER.Write(Log::INFO, "ModelPositionController::Update()");

    if (_poseList.size() == 0)
	{
        gzthrow("ModelPositionController::Update() PoseList size == 0!");
		return;
	}
	
    if (_registerFirstLoop == false)
	{
        this->_lastUpdateTime = info.simTime;

        _registerFirstLoop = true;
    }
	
    gazebo::common::Time dt = info.simTime - _lastUpdateTime;

    this->_lastUpdateTime = info.simTime;

	_timeAcum += dt.Double();

    //
    // Updating current target pos
    //
    if (_timeAcum > _timeStep)
    {
        //LOGGER.Write(Log::INFO, "ModelPositionController::Update() delta time : %f", dt);
        while (_timeAcum > _timeStep)
        {
            _timeAcum = _timeAcum - _timeStep;

            _currentPoseIndex++;

            CheckSimulationReachedEnd();
        }
    }

	
	gazebo::math::Vector3 currentTargetPos;
	gazebo::math::Vector3 currentTargetRotation;

    gazebo::math::Vector3 targetLinearAcceleration;
    gazebo::math::Vector3 targetAngularAcceleration;

    std::stringstream ss;

    if ((_currentPoseIndex >= 1) && (this->_interpolatePose))
	{
        //
        // Interpolação Linear
        //

        float interpFactor = _timeAcum / _timeStep;

        //LOGGER.Write(Log::INFO, "ModelPositionController::Update() dt : %f | timeStep : %f | interpFactor %f | currentPoseIndex : %d ", dt, timeStep, interpFactor, currentPoseIndex );

        //if (interpFactor >= 1.0f)
        //	interpFactor = 1.0f;

		gazebo::math::Vector3 currentPos;
		gazebo::math::Vector3 lastPos;

        gazebo::math::Vector3 currentLinearAccel;
        gazebo::math::Vector3 lastLinearAccel;

        //Pos linear
        currentPos.x = _poseList[_currentPoseIndex]->pos_x;
        currentPos.y = _poseList[_currentPoseIndex]->pos_y;
        currentPos.z = _poseList[_currentPoseIndex]->pos_z;

        lastPos.x = _poseList[_currentPoseIndex - 1]->pos_x;
        lastPos.y = _poseList[_currentPoseIndex - 1]->pos_y;
        lastPos.z = _poseList[_currentPoseIndex - 1]->pos_z;

        //Accel Linear
        currentLinearAccel.x = _accelerationList[_currentPoseIndex]->pos_x;
        currentLinearAccel.y = _accelerationList[_currentPoseIndex]->pos_y;
        currentLinearAccel.z = _accelerationList[_currentPoseIndex]->pos_z;

        lastLinearAccel.x = _accelerationList[_currentPoseIndex - 1]->pos_x;
        lastLinearAccel.y = _accelerationList[_currentPoseIndex - 1]->pos_y;
        lastLinearAccel.z = _accelerationList[_currentPoseIndex - 1]->pos_z;

        //PosiçãoAngular
		gazebo::math::Vector3 currentRot;
		gazebo::math::Vector3 lastRot;

        currentRot.x = _poseList[_currentPoseIndex]->rot_x;
        currentRot.y = _poseList[_currentPoseIndex]->rot_y;
        currentRot.z = _poseList[_currentPoseIndex]->rot_z;

        lastRot.x = _poseList[_currentPoseIndex - 1]->rot_x;
        lastRot.y = _poseList[_currentPoseIndex - 1]->rot_y;
        lastRot.z = _poseList[_currentPoseIndex - 1]->rot_z;

        //Acceleração Angular
        gazebo::math::Vector3 currentAngularAccel;
        gazebo::math::Vector3 lastAngularAccel;

        currentAngularAccel.x = _accelerationList[_currentPoseIndex]->rot_x;
        currentAngularAccel.y = _accelerationList[_currentPoseIndex]->rot_y;
        currentAngularAccel.z = _accelerationList[_currentPoseIndex]->rot_z;

        lastAngularAccel.x = _accelerationList[_currentPoseIndex - 1]->rot_x;
        lastAngularAccel.y = _accelerationList[_currentPoseIndex - 1]->rot_y;
        lastAngularAccel.z = _accelerationList[_currentPoseIndex - 1]->rot_z;

        //Interpolação Linear
		currentTargetPos = lastPos * (1.0f - interpFactor) + currentPos * interpFactor;
		currentTargetRotation = lastRot * (1.0f - interpFactor) + currentRot * interpFactor;

        targetLinearAcceleration = lastLinearAccel * (1.0f - interpFactor) + currentLinearAccel * interpFactor;
        targetAngularAcceleration = lastAngularAccel * (1.0f - interpFactor) + currentAngularAccel * interpFactor;

	}
	else
	{
        currentTargetPos = gazebo::math::Vector3(	_poseList[_currentPoseIndex]->pos_x,
                                                    _poseList[_currentPoseIndex]->pos_y,
                                                    _poseList[_currentPoseIndex]->pos_z);

        currentTargetRotation = gazebo::math::Vector3(	_poseList[_currentPoseIndex]->rot_x,
                                                        _poseList[_currentPoseIndex]->rot_y,
                                                        _poseList[_currentPoseIndex]->rot_z);

        targetLinearAcceleration = gazebo::math::Vector3(	_accelerationList[_currentPoseIndex]->pos_x,
                                                        _accelerationList[_currentPoseIndex]->pos_x,
                                                        _accelerationList[_currentPoseIndex]->pos_x);

        targetAngularAcceleration = gazebo::math::Vector3(	_accelerationList[_currentPoseIndex]->rot_x,
                                                        _accelerationList[_currentPoseIndex]->rot_y,
                                                        _accelerationList[_currentPoseIndex]->rot_z);
	}

    auto worldPos = _model->GetWorldPose();
    auto worldLinearVel = _model->GetWorldLinearVel();
    auto worldAngularVel = _model->GetWorldAngularVel();
    auto worldLinearAcc = _model->GetWorldLinearAccel();
    auto worldAngularAcc = _model->GetWorldAngularAccel();
    auto worldPosRot = worldPos.rot.GetAsEuler();

    ss << info.simTime.Double() << " ";
    ss << worldPos.pos.x << " " << worldPos.pos.y << " " << worldPos.pos.z << " ";
    ss << worldPosRot.x << " " << worldPosRot.y << " " << worldPosRot.z << " ";
    ss << worldLinearVel.x << " " << worldLinearVel.y << " " << worldLinearVel.z << " ";
    ss << worldAngularVel.x << " " << worldAngularVel.y << " " << worldAngularVel.z << " ";
    ss << worldLinearAcc.x << " " << worldLinearAcc.y << " " << worldLinearAcc.z << " ";
    ss << worldAngularAcc.x << " " << worldAngularAcc.y << " " << worldAngularAcc.z << " ";

    ss << currentTargetPos.x << " " << currentTargetPos.y << " " << currentTargetPos.z << " ";
    ss << currentTargetRotation.x << " " << currentTargetRotation.y << " " << currentTargetRotation.z << " ";
    ss << targetLinearAcceleration.x << " " << targetLinearAcceleration.y << " " << targetLinearAcceleration.z << " ";
    ss << targetAngularAcceleration.x << " " << targetAngularAcceleration.y << " " << targetAngularAcceleration.z << " ";


		
	if (_useController)
	{
		gazebo::math::Vector3 errorPosition = GetPositionError(currentTargetPos);
		gazebo::math::Vector3 errorRotation = GetRotationError(currentTargetRotation);

		//LOGGER.Write(Log::INFO, "ModelPositionController::Update() - Error Pos [%f, %f, %f] Rot [%f, %f, %f]",
		//	errorPosition.x, errorPosition.y, errorPosition.z,
		//	errorRotation.x, errorRotation.y, errorRotation.z);

        gazebo::math::Vector3 forceCommandWorldSpace = gazebo::math::Vector3::Zero;
        gazebo::math::Vector3  torqueCommandWorldSpace = gazebo::math::Vector3::Zero;

        if (_controllerType == "sliding")
        {
            //
            // SLIDING
            //

            for (int i = 0; i < 6; i++)
            {
                _controlledAxisArray[i]._slidingMode->_k = _controlledAxisArray[i]._k;
                _controlledAxisArray[i]._slidingMode->_a = _controlledAxisArray[i]._a;
            }

            // Axis X
            _controlledAxisArray[0]._slidingMode->Update(errorPosition.x, dt);
            // Axis Y
            _controlledAxisArray[1]._slidingMode->Update(errorPosition.y, dt);
            // Axis Z
            _controlledAxisArray[2]._slidingMode->Update(errorPosition.z, dt);
            // Axis rotation X
            _controlledAxisArray[3]._slidingMode->Update(errorRotation.x, dt);
            // Axis rotation Y
            _controlledAxisArray[4]._slidingMode->Update(errorRotation.y, dt);
            // Axis rotation Z
            _controlledAxisArray[5]._slidingMode->Update(errorRotation.z, dt);


            forceCommandWorldSpace = gazebo::math::Vector3((float)_controlledAxisArray[0]._slidingMode->GetCmd(),
                (float)_controlledAxisArray[1]._slidingMode->GetCmd(),
                (float)_controlledAxisArray[2]._slidingMode->GetCmd());

            torqueCommandWorldSpace = gazebo::math::Vector3((float)_controlledAxisArray[3]._slidingMode->GetCmd(),
                (float)_controlledAxisArray[4]._slidingMode->GetCmd(),
                (float)_controlledAxisArray[5]._slidingMode->GetCmd());
        }
        else{
            //
            //  PID
            //

            for (int i = 0; i < 6; i++)
            {
            _controlledAxisArray[i]._pid->SetPGain(_controlledAxisArray[i]._p);
            _controlledAxisArray[i]._pid->SetIGain(_controlledAxisArray[i]._i);
            _controlledAxisArray[i]._pid->SetDGain(_controlledAxisArray[i]._d);
            }

            // Axis X
            _controlledAxisArray[0]._pid->Update(errorPosition.x, dt);
            // Axis Y
            _controlledAxisArray[1]._pid->Update(errorPosition.y, dt);
            // Axis Z
            _controlledAxisArray[2]._pid->Update(errorPosition.z, dt);
            // Axis rotation X
            _controlledAxisArray[3]._pid->Update(errorRotation.x, dt);
            // Axis rotation Y
            _controlledAxisArray[4]._pid->Update(errorRotation.y, dt);
            // Axis rotation Z
            _controlledAxisArray[5]._pid->Update(errorRotation.z, dt);


            forceCommandWorldSpace = gazebo::math::Vector3((float)_controlledAxisArray[0]._pid->GetCmd(),
            (float)_controlledAxisArray[1]._pid->GetCmd(),
            (float)_controlledAxisArray[2]._pid->GetCmd());

            torqueCommandWorldSpace = gazebo::math::Vector3((float)_controlledAxisArray[3]._pid->GetCmd(),
            (float)_controlledAxisArray[4]._pid->GetCmd(),
            (float)_controlledAxisArray[5]._pid->GetCmd());

            // A classe PID do Gazebo usa uma convenção de sinal diferente, então é preciso invertê-lo.
            // https://bitbucket.org/osrf/gazebo/src/7522abe667055b3a3da61796270f7fb87467d81e/gazebo/common/PID.cc?at=default&fileviewer=file-view-default
            // linha 151
            forceCommandWorldSpace *= -1;
            torqueCommandWorldSpace *= -1;
        }

        ss << errorPosition.x << " " << errorPosition.y << " " << errorPosition.z << " ";


		//
		// Applying force and torque
		//

        // Aqui são aplicadas duas forças/torques a primiera calculada pelo controlador feeedback e a segunda
        //calculada apartir da lista de acelerações que o navio devia estar sofrendo naquele devido instante.
        // Como a API do Gazebo não permite aplicar acelerações diretamente e o
        // referencial é estático, é feita uma aproximação:
        // mass * acceleration  = F_aplicada + err_desprezado
        // Inertia * angularAccleration = T_aplicado + err_desprezado
        const gazebo::physics::Link_V links = _model->GetLinks();

        if (_model->GetLinks().size() > 0)
		{
            double precalculatedForces; // 0 ou 1
            if (this->_usePrecalculatedForces)
            {
                precalculatedForces = 1;
            }
            else
            {
                precalculatedForces = 0;
            }

            double mass = links[0]->GetInertial()->GetMass();
            math::Matrix3 inertialMatrix = links[0]->GetInertial()->GetMOI(links[0]->GetInertial()->GetPose());

            links[0]->AddForce(forceCommandWorldSpace + mass * precalculatedForces * targetLinearAcceleration);

            links[0]->AddTorque(torqueCommandWorldSpace + inertialMatrix * precalculatedForces * targetAngularAcceleration);
        }
	}
    else //Usando apenas SetPosition()
	{
        SetModelPosition(_currentPoseIndex);
    }
}


gazebo::math::Vector3 ModelPositionController::GetPositionError( gazebo::math::Vector3 targetPosition )
{
	gazebo::math::Vector3 errorPos;
    errorPos = targetPosition - _model->GetWorldPose().pos;

	return errorPos;
}


gazebo::math::Vector3 ModelPositionController::GetRotationError(gazebo::math::Vector3 targetRotation)
{
	gazebo::math::Vector3 newTargetRotation = targetRotation;
	
    gazebo::math::Quaternion currentRotationInverted = _model->GetWorldPose().rot;
	currentRotationInverted.Invert();

	gazebo::math::Quaternion errorRotationQuaternion = gazebo::math::Quaternion::EulerToQuaternion(newTargetRotation) * currentRotationInverted;
	gazebo::math::Vector3 errorRot = errorRotationQuaternion.GetAsEuler();

    //Manter sinal de erro entre -PI e +PI, já que desejamos que o erro esteja em torno de ZERO.
	if (errorRot.x > gazebo::math::Angle::Pi.Radian() )
		errorRot.x = errorRot.x - gazebo::math::Angle::TwoPi.Radian();
	else if (errorRot.x < - gazebo::math::Angle::Pi.Radian())
		errorRot.x = errorRot.x + gazebo::math::Angle::TwoPi.Radian();

	if (errorRot.y > gazebo::math::Angle::Pi.Radian())
		errorRot.y = errorRot.y - gazebo::math::Angle::TwoPi.Radian();
	else if (errorRot.y < - gazebo::math::Angle::Pi.Radian())
		errorRot.y = errorRot.y + gazebo::math::Angle::TwoPi.Radian();

	if (errorRot.z > gazebo::math::Angle::Pi.Radian())
		errorRot.z = errorRot.z - gazebo::math::Angle::TwoPi.Radian();
	else if (errorRot.z < - gazebo::math::Angle::Pi.Radian())
        errorRot.z = errorRot.z + gazebo::math::Angle::TwoPi.Radian();

	return errorRot;
}
