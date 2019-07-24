#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF2_core::controlGpsYawReset()
{

    if (yawControl){	
		if (lastTimeGpsHeadReceived_ms != 0){
			if (AP_HAL::millis() - lastTimeGpsHeadReceived_ms > 1000){
				lastTimeGpsHeadLost_ms = AP_HAL::millis();
					
				//Dual-antenna GPS error until the double-antenna correction yaw function is turned off before the power is turned off.
				yawControl = 0;
				isGpsYawFusion = false;

				//Use magnetic compass data to force reset yaw angle
				magYawResetRequest = true;
				magStateResetRequest = true;
				frontend->gps_yaw_health = false;
				gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u GPS yaw anomaly, yaw re-aligned for mag",(unsigned)imu_index);
			}
		}
	}
	else{
		if (lastTimeGpsHeadLost_ms != 0 && AP_HAL::millis() - lastTimeGpsHeadLost_ms > 10000){
			if (AP_HAL::millis() - lastTimeGpsHeadReceived_ms < 220){
				//Dual antenna GPS restores health and re-enables dual antenna orientation.
				yawControl = 1;
                isGpsYawFusion = true;
				
                alignGpsYaw();
				
				frontend->gps_yaw_health = true;
				gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u GPS yaw restores health",(unsigned)imu_index);
			}
		}
	}

	if (yawControl){	
		if (lastTimeGpsHeadReceived_ms != 0){	
			if (prelastTimeGpsHeadReceived_ms == 0 && lastTimeGpsHeadReceived_ms != 0){
				shouldResetYaw = true;
			}
			
			if (gpsHeadDataToFuse && yawControl){
				if (shouldResetYaw){
					shouldResetYaw = false;
					isGpsYawFusion = true;
					alignGpsYaw();
				}
			}
		}
	}

	prelastTimeGpsHeadReceived_ms = lastTimeGpsHeadReceived_ms;
}


void NavEKF2_core::alignGpsYaw()
{
    // get the euler angles from the current state estimate
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    //baiyang added in 20170116
    float gps_yaw = wrap_PI(yawAngDataDelayed.yawAng);
	gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u GPS yaw %f",(unsigned)imu_index,degrees(yawAngDataDelayed.yawAng));

	// Use the Euler angles and gps yaw measurement to update the magnetic field states
    // and get an updated quaternion
    Quaternion newQuat;
    newQuat.from_euler(eulerAngles.x, eulerAngles.y, gps_yaw);
	zeroAttCovOnly();

    // clear any pending yaw reset requests
    gpsYawResetRequest = false;
    magYawResetRequest = false;
	
	// previous value used to calculate a reset delta
    Quaternion prevQuat = stateStruct.quat;

    // update the quaternion states using the new yaw angle
    stateStruct.quat = newQuat;

    // calculate the change in the quaternion state and apply it to the ouput history buffer
    prevQuat = stateStruct.quat/prevQuat;
    StoreQuatRotate(prevQuat);

    // record the yaw reset event
    recordYawReset();
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF2_core::SelectGpsYawFusion()
{
    // start performance timer
    //hal.util->perf_begin(_perf_FuseMagnetometer);
	
	// check for availability of gps heading data to fuse
	gpsHeadDataToFuse = storedYawAng.recall(yawAngDataDelayed,imuDataDelayed.time_ms);

    controlGpsYawReset();

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (gpsHeadDataToFuse && yawControl && yawAlignComplete);
    if (dataReady) {
        fuseGpsEulerYaw();
        // zero the test ratio output from the inactive 3-axis magnetometer fusion
        magTestRatio.zero();
        yawTestRatio = 0.0f;
    }
	
    // stop performance timer
    //hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the magnetic poles)
*/
void NavEKF2_core::fuseGpsEulerYaw()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2)
	const float R_YAW = sq(yawAngDataDelayed.yawAngErr);

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    // determine if a 321 or 312 Euler sequence is best
    float predicted_yaw;
    float H_YAW[3];
    if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
        // calculate observation jacobian when we are observing the first rotation in a 321 sequence
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2+t3-t4-t5;
        float t7 = q0*q3*2.0f;
        float t8 = q1*q2*2.0f;
        float t9 = t7+t8;
        float t10 = sq(t6);
        if (t10 > 1e-6f) {
            t10 = 1.0f / t10;
        } else {
            return;
        }
        float t11 = t9*t9;
        float t12 = t10*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = 0.0f;
        H_YAW[1] = t14*(t15*(q0*q1*2.0f-q2*q3*2.0f)+t9*t10*(q0*q2*2.0f+q1*q3*2.0f));
        H_YAW[2] = t14*(t15*(t2-t3+t4-t5)+t9*t10*(t7-t8));

        // calculate predicted and measured yaw angle
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        predicted_yaw = euler321.z;
    } else {
        // calculate observation jacobian when we are observing a rotation in a 312 sequence
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2-t3+t4-t5;
        float t7 = q0*q3*2.0f;
        float t10 = q1*q2*2.0f;
        float t8 = t7-t10;
        float t9 = sq(t6);
        if (t9 > 1e-6f) {
            t9 = 1.0f/t9;
        } else {
            return;
        }
        float t11 = t8*t8;
        float t12 = t9*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = -t14*(t15*(q0*q2*2.0+q1*q3*2.0)-t8*t9*(q0*q1*2.0-q2*q3*2.0));
        H_YAW[1] = 0.0f;
        H_YAW[2] = t14*(t15*(t2+t3-t4-t5)+t8*t9*(t7+t10));

        // calculate predicted and measured yaw angle
        Vector3f euler312 = stateStruct.quat.to_vector312();
        predicted_yaw = euler312.z;
    }

    // Calculate the innovation
    float innovation = wrap_PI(predicted_yaw - yawAngDataDelayed.yawAng);

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
    if (varInnov >= R_YAW) {
        varInnovInv = 1.0f / varInnov;
        // output numerical health status
        faultStatus.bad_yaw = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        // output numerical health status
        faultStatus.bad_yaw = true;
        return;
    }

    // calculate Kalman gain
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * varInnov);

    // Declare the magnetometer unhealthy if the innovation test fails
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        // On the ground a large innovation could be due to large initial gyro bias or magnetic interference from nearby objects
        // If we are flying, then it is more likely due to a magnetometer fault and we should not fuse the data
        if (inFlight) {
            return;
        }
    } else {
        magHealth = true;
    }

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    // calculate K*H*P
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= 2; column++) {
            KH[row][column] = Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            float tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            KHP[row][column] = tmp;
        }
    }

    // Check that we are not going to drive any variances negative and skip the update if so
    bool healthyFusion = true;
    for (uint8_t i= 0; i<=stateIndexLim; i++) {
        if (KHP[i][i] > P[i][i]) {
            healthyFusion = false;
        }
    }
    if (healthyFusion) {
        // update the covariance matrix
        for (uint8_t i= 0; i<=stateIndexLim; i++) {
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }

        // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
        ForceSymmetry();
        ConstrainVariances();

        // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t i=0; i<=stateIndexLim; i++) {
            statesArray[i] -= Kfusion[i] * innovation;
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

        // record fusion event
        faultStatus.bad_yaw = false;
        lastYawTime_ms = imuSampleTime_ms;


    } else {
        // record fusion numerical health status
        faultStatus.bad_yaw = true;
    }
}

