// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.content.*;
import android.app.PendingIntent;
import android.hardware.usb.*;
import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
import android.util.Log;
import android.os.SystemClock;

class InfiniTAMUSBPermissionListener extends BroadcastReceiver {
	private UsbDevice device;
	private boolean granted;
	private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

	public InfiniTAMUSBPermissionListener() {
		granted = false;
		device = null;
	}

	public void onReceive(Context context, Intent intent) {
		String action = intent.getAction();
		if (!ACTION_USB_PERMISSION.equals(action)) return;

		synchronized (this) {
			UsbDevice tmp_device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

			if (!intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) return;
			if (tmp_device == null) return;

			device = tmp_device;
			granted = true;
		}
	}

	public boolean permissionGranted()
	{ return granted; }

	public UsbDevice getDevice()
	{ return device; }
}

class InfiniTAMRenderer implements GLSurfaceView.Renderer
{
	public void onDrawFrame(GL10 gl) 
	{
		RenderGL();
	}

	public void onSurfaceChanged(GL10 gl, int width, int height) 
	{		
		ResizeGL(width, height);
	}

	public void onSurfaceCreated(GL10 gl, EGLConfig config) 
	{
		InitGL();
	}

	public static native void InitGL();
	public static native void ResizeGL(int newWidth, int newHeight);
	public static native void RenderGL();
}

class InfiniTAMProcessor implements Runnable
{
	private InfiniTAMView view;
	private InfiniTAMUSBPermissionListener listener;
	private UsbManager usbManager;

	InfiniTAMProcessor(UsbManager _usbManager, InfiniTAMUSBPermissionListener _listener)
	{
		view = null;
		listener = _listener;
		usbManager = _usbManager;
	}

	public void attachVisualisation(InfiniTAMView _view)
	{
		view = _view;
	}

	public void detachVisualisation(InfiniTAMView _view)
	{
		view = null;
	}

	public void run()
	{
		int camera_fd = waitForUSBPermission();

		StartProcessing((camera_fd >= 0)?1:0);

		while (true) {
			int ret = ProcessFrame();
			if (view != null) view.requestRender();
			if (ret == 0) break;
		}
	}


	int waitForUSBPermission()
	{
		if (listener == null) return -1;
		while (!listener.permissionGranted()) {
			SystemClock.sleep(100);
		}

		UsbDevice depthCamera = listener.getDevice();
		UsbDeviceConnection con = usbManager.openDevice(depthCamera);
		return con.getFileDescriptor();
	}

	public static native void StartProcessing(int useLiveCamera);
	public static native int ProcessFrame();
}

