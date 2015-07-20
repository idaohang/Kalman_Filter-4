#include "filter.h"



filter::filter(void){};
filter::~filter(void){};



matDouble filter::stateUpdate()
{
	return stateUpdate(_transitionMatrix, _controlMatrix, _systemState_k, _control_k, _processNoise_k);
}


matDouble filter::measurementUpdate()
{
	return measurementUpdate(_measurement_k, _systemState_k, _measurementNoise_k);
}


matDouble filter::buildMeasurementErrCov()
{
	return la.outerProduct(_measurementNoise_k, _measurementNoise_k);
}


matDouble filter::buildProcessNoiseCov()
{

	return la.outerProduct(_processNoise_k, _processNoise_k);
}



void filter::updateEstimationErrorConvariance()
{
	_estimationErrCov_k = _transitionMatrix * _estimationErrCov_k * la.tranpose(_transitionMatrix) + \
		_processNoise_k - _transitionMatrix * _processNoise_k * la.tranpose(_controlMatrix)* la.inverseLU(_measurementCov) * \
		_controlMatrix * _estimationErrCov_k * la.tranpose(_transitionMatrix);
}


void filter::updateKalmanGain()
{
	_kalmanGain_k = _transitionMatrix * _estimationErrCov_k * la.tranpose(_controlMatrix) *   \
		la.inverseLU(( _controlMatrix * _estimationErrCov_k * la.tranpose(_controlMatrix) + _measurementCov)) ;
}


void filter::updateStateEstimate()
{
	_stateEstimate_k = (_transitionMatrix * _stateEstimate_k + _controlMatrix * _control_k) + \
		_kalmanGain_k*(_measurement_k - _stateEstimate_k);
}



//----------------------------------------- 1D Kalman Filter Example --------------------------------------------------------///

matrix<double> filter::buildTransitionMatrix_1D()
{
	matDouble result(3, 3);
	result(1, 1) = 1; result(1, 2) = _updateTimeStep;
	result(2, 1) = 0; result(2, 2) = 1;
	return result;
}

matrix<double> filter::buildMeasurementMatrix_1D()
{
	matDouble result(1, 2);
	result(1, 1) = 1; result(1, 2) = 0;
	return result;
}

matrix<double> filter::buildControlMatrix_1D()
{
	matDouble result(1, 2);
	result(1, 1) = _updateTimeStep*_updateTimeStep; result(1, 2) = _updateTimeStep;
	return result;
}
