#ifndef MODEL_POSITION_CONTROLLER_H
#define MODEL_POSITION_CONTROLLER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>

namespace gazebo
{
	class GAZEBO_VISIBLE ModelPositionController : public  ModelPlugin
	{
		struct PoseInfo
		{
			double pos_x;
			double pos_y;
			double pos_z;

			double rot_x;
			double rot_y;
			double rot_z;

			PoseInfo() : pos_x(0), pos_y(0), pos_z(0), rot_x(0), rot_y(0), rot_z(0)
			{
			}
		};

        /// <summary>
        /// Classe que representa um controlador por Modos Deslizantes Simplificado.
        /// </summary>
		class SlidingMode
		{
		public:

			/// \brief Error at a previous step.
			double _errLast;
			/// \brief Current error.
			double _err;
			/// \brief Current error derivative.
			double _errDerivative;
			/// \brief Gain of the control signal.
			double _k;
			/// \brief A gain that define how fast the error reaches zero.
			double _a;
			/// \brief Command value.
			double _cmd;

			SlidingMode(double k, double a)
			{
                 Init(k,a);
			}

			void Init(double k, double a)
			{
				_k = k;
				_a = a;

				Reset();
			}

			void Reset()
			{
				_errLast = 0.0;
				_err = 0.0;
				_cmd = 0.0;
			}

			double Update(double error, common::Time dt)
			{
                _err = error;

				if (dt == common::Time(0, 0) || ignition::math::isnan(error) || std::isinf(error))
					return 0.0;

				if (dt != common::Time(0, 0))
				{
					_errDerivative = (_err - _errLast) / dt.Double();
					_errLast = _err;
				}

				double sigma = _a * _err + _errDerivative;

				_cmd = _k * Sigmoid(sigma); // Controle foi modificado. Em vez de usar a função Sign usa-se uma função sigmóide para diminuir as oscilações em regime permanente.

                //if (sigma >= 0.0)
				//	_cmd = _k;
				//else
				//	_cmd = -_k;

				return _cmd;
			}

			double GetCmd()
			{
				return _cmd;
			}

			/// <summary>
			/// Sigmoid modificada para a necessidade especifica dessa classe.
			/// </summary>
			/// <param name="x">Input da funcao</param>
			/// <returns>Output da funcao</returns>
			double Sigmoid(double x)
			{
				return 2.0*((1.0 / (1.0 + std::exp(-x*0.35))) - (1.0 / 2.0));
			}
		};

		class ControlledAxis
		{
			public:
			
				float _p, _i, _d;
				gazebo::common::PID *_pid;

				float _k, _a;
				SlidingMode *_slidingMode;

			ControlledAxis()
			{
				_k = 4800; 
				_a = 10;

                _p = 4800;
                _i = 0;
                _d = 0;
			}

			void Init(float p, float i, float d)
			{
				_p = p;
				_i = i;
				_d = d;

                _pid = new gazebo::common::PID((double)p, (double)i, (double)d, 100000000, -100000000, 100000000, -100000000);
			}

			void Init(float k, float a)
			{
				_k = k;
				_a = a;

				_slidingMode = new SlidingMode(_k, _a);
			}
		};



		public: ModelPositionController();
		public: virtual ~ModelPositionController();
		
		public:	virtual void Reset();
        public: virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        private: virtual void Update(const common::UpdateInfo &info);

        private: void SetModelPosition(int index); //Move o modelo para na posição index do vetor _poseList
        private: void ResetModelToStartPose();
        private: void CheckSimulationReachedEnd(); //Checa se simulação alcançou fim do arquivo. Caso tenha, RESETA Gazebo.

		public:	 gazebo::math::Vector3 GetPositionError(gazebo::math::Vector3 targetPosition );
		public:	 gazebo::math::Vector3 GetRotationError(gazebo::math::Vector3 targetRotation);

        private: physics::ModelPtr _model;
        private: physics::WorldPtr _world;

        private: event::ConnectionPtr _updateConnection;

        public: int _totalPoses;
        public: unsigned int _currentPoseIndex;
        public: double _timeStep;
        public: double _startTime = 0.0; // Tempo em segundos de onde começar a ler o arquivo de simulação.

        public: bool _interpolatePose; // Se true, faz interpolação linear entre poses da lista.
        public: bool _useController; // Se true, usa controlador. Se false, altera posição diretamente.
        public: std::string _controllerType; // Pode ser PID (pid), ou Modos Deslizantes (sliding).
        public: bool _usePrecalculatedForces = true; // Se true, usa forças pre calculadas do arquivo txt.

        public: gazebo::common::Time _lastUpdateTime;
        public: bool _registerFirstLoop;
        public: double _timeAcum; // Tempo em segundos usado para a interpolação. Quando maior que _timeStep,
                                  //passe-se para a pose seguinte de _poseList.

        public: std::vector<PoseInfo*>  _poseList; // lista de poses vindas do arquivo de simulação da embarcação. Rotações em radianos.
        public: std::vector<PoseInfo*>  _accelerationList;// lista de acelerações vindas do arquivo de simulação da embarcação. Rotações em radianos.
		public:	ControlledAxis _controlledAxisArray[6];
	};
}
#endif 
