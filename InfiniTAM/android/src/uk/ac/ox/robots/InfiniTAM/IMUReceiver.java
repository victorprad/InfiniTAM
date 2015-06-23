// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.content.Context;

import android.util.Log;

public class IMUReceiver implements SensorEventListener {
	private final SensorManager mSensorManager;
	private final Sensor mGyroscope;
	private final Sensor mAccelerometer;
	private final Sensor mMagnetometer;

	public IMUReceiver(Context ctx) {
		mSensorManager = (SensorManager)ctx.getSystemService(ctx.SENSOR_SERVICE);
		mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE/*_UNCALIBRATED*/);
		mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		mMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD/*_UNCALIBRATED*/);
	}

	public void start() {
		mSensorManager.registerListener(this, mGyroscope, 10000);
		mSensorManager.registerListener(this, mAccelerometer, 10000);
		mSensorManager.registerListener(this, mMagnetometer, 10000);
	}

	public void stop() {
		mSensorManager.unregisterListener(this);
	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// ignore...
	}

	public void onSensorChanged(SensorEvent event) {
		if (event.sensor == mGyroscope) {
			Log.v("IMUReciever", "gyr "+event.values[0]+" "+event.values[1]+" "+event.values[2]);
		} else if (event.sensor == mAccelerometer) {
			Log.v("IMUReciever", "acc "+event.values[0]+" "+event.values[1]+" "+event.values[2]);
		} else if (event.sensor == mMagnetometer) {
			Log.v("IMUReciever", "mag "+event.values[0]+" "+event.values[1]+" "+event.values[2]);
		} else {
			Log.v("IMUReciever", "rubbish received");
		}
	}
 }

