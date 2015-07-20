

// Will Implement Different Filter
/*
	The Class will include a while Loop which will take in the state and give the filtered output at each point of time 
	1 D Kalman Filter 
*/

// System
#include <iostream>




// Mine
#include "func.h" 
#include "la_pack.h" 
#include "analysis.h"


class filter
{
public:
	filter(void);
	~filter(void);


	// Use only After Initializationg of System 
	void updateEstimationErrorConvariance();
	void updateKalmanGain();
	void updateStateEstimate();




private:
	la_pack la; 

	// Fixed System Parameters
	matDouble _transitionMatrix; 
	matDouble _measurementMatrix;
	matDouble _controlMatrix;


	// Varying System Parameters 
	matDouble _velocity_k; 
	matDouble _velocityNoise_k;
	matDouble _position_k; 
	matDouble _positionNoise_k;
	matDouble _control_k;
	matDouble _processNoise_k;
	matDouble _measurement_k ,_measurementNoise_k;
	matDouble _systemState_k; // = la.createBlockColVec_2(v_k, p_k); // do this within the constructor !
	
	// Estimation 
	matDouble _stateEstimate_k; // The state estimated by Kalman Filter 
	matDouble _estimationErrCov_k; 
	matDouble _kalmanGain_k; 


	// Noise Convariance 
	matDouble _measurementCov; 
	matDouble _processCov;


	// Update Step 
	double _updateTimeStep; // Sensor reading per unit time. 

	/*
		Build Corresponding Matrix ; 
		Used in the initialization of the Filter 
	*/

	matDouble buildMeasurementErrCov(); //Returns the measurement error Convariance Matrix 
	matDouble buildProcessNoiseCov();//Returns the measurement error Convariance Matrix
	


	matDouble stateUpdate(); 
	matDouble measurementUpdate();
	

	//1 D Kalman Filter Example 
	matrix<double> buildTransitionMatrix_1D();
	matrix<double> buildMeasurementMatrix_1D();
	matrix<double> buildControlMatrix_1D();


	// CONVENIENCE FUNCTIONS 

	/*
		FORM  :: x_k+1 = A * x_k + B * u_k + w_k 
		Describes the state of the system 
		k -> Time Index 
		u -> input to system 
		y -> measured output 
		w -> process noise
		
	*/
	matrix<double> stateUpdate(matDouble A, matDouble B, matDouble systemState_k, matDouble control_k, matDouble processNoise_k){ return A * systemState_k + B*control_k + processNoise_k; };
	
	/*
		FORM :: y_k = C * x_k + z_k 
		Describes the measurements of system 
		k -> Time Index
		y -> Output 
		z -> measurement noise
		*/
	matrix<double> measurementUpdate(matDouble C, matDouble systemState_k, matDouble measurementNoise_k){ return C *systemState_k + measurementNoise_k; };
	
	/*
		FORM :: v_k+1 = v_k + T * u_k ; 
		Describes velocity evolution in time  
		v_k+1 = v_k + T * u_k + v_k_n; 

		v_k_n -> velocity Noise 
	*/
	matrix<double> velocityUpdate(matDouble velocity_k, matDouble T, matDouble control_k, matDouble velocityNoise_k) { return velocity_k + T * control_k + velocityNoise_k; };
	
	/*
		FORM :: p_k+1 = p_k + T * v_k + (1/2) * T *T * u_k + p_k_n 
	*/
	matrix<double> positionUpdate(matDouble position_k, matDouble T, matDouble velocity_k, matDouble control_k, matDouble positionNoise_k) { return position_k + T * velocity_k + T * T * (1 / 2) * control_k + positionNoise_k; }





};