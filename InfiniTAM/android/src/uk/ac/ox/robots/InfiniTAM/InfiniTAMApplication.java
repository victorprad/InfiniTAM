// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

//import android.content.*;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.app.Application;
import android.app.PendingIntent;
import android.hardware.usb.*;
import java.util.HashMap;
import java.util.Iterator;
import java.lang.Thread;
import android.util.Log;
import uk.ac.ox.robots.InfiniTAM.InfiniTAMProcessor;

public class InfiniTAMApplication extends Application
{   
	static {
		System.loadLibrary("InfiniTAM");
	}

	private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

	private static InfiniTAMApplication s_instance;
	private Thread processingThread;
	private InfiniTAMProcessor processor;
	private IMUReceiver imuReceiver;

	public InfiniTAMApplication()
	{
		s_instance = this;
	}

	public void onCreate()
	{
		super.onCreate();
		InitializeNativeApp(getApplicationInfo().nativeLibraryDir);

		//imuReceiver = new IMUReceiver(this);
		//imuReceiver.start();

		InfiniTAMUSBPermissionListener listener = askForUSBPermission();

		processor = new InfiniTAMProcessor((UsbManager)getSystemService(Context.USB_SERVICE), listener);
		processingThread = new Thread(processor);
		processingThread.start();
	}

	public static InfiniTAMApplication getApplication()
	{
		return s_instance;
	}

	public InfiniTAMProcessor getProcessor()
	{
		return processor;
	};

	InfiniTAMUSBPermissionListener askForUSBPermission()
	{
		UsbManager manager = (UsbManager)getSystemService(Context.USB_SERVICE);

		// register someone to listen for "permission granted" dialogs
		PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		InfiniTAMUSBPermissionListener listener = new InfiniTAMUSBPermissionListener();
		registerReceiver(listener, filter);

		// get a ist of USB devices
		HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
		Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();

		// find the depth camera
		UsbDevice depthCamera = null;
		while (deviceIterator.hasNext()) {
			UsbDevice device = deviceIterator.next();

			if ((device.getVendorId() == 0x1d27) &&
			    (device.getProductId() == 0x0600)) {
				depthCamera = device;
			}
		}
		if (depthCamera == null) return null;

		manager.requestPermission(depthCamera, permissionIntent);
		return listener;
	}

	public static native void InitializeNativeApp(String libdir);
}

