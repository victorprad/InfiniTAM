// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

//import android.content.*;
import android.app.Application;
import android.app.PendingIntent;
import android.content.*;
import android.hardware.usb.*;
import java.util.HashMap;
import java.util.Iterator;
import android.os.SystemClock;

public class InfiniTAMApplication extends Application
{   
	static {
		System.loadLibrary("InfiniTAM");
	}

	public static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

	private static InfiniTAMApplication s_instance = null;
	private InfiniTAMUSBPermissionListener usbListener;

	public InfiniTAMApplication()
	{
		if (s_instance == null) s_instance = this;
	}

	public void onCreate()
	{
		super.onCreate();
		InitializeNativeApp(getApplicationInfo().nativeLibraryDir);

		askForUSBPermission();

		//imuReceiver = new IMUReceiver(this);
		//imuReceiver.start();
	}

	public static InfiniTAMApplication getApplication()
	{
		return s_instance;
	}

	// return true if a device is found
	boolean askForUSBPermission()
	{
		UsbManager manager = (UsbManager)getSystemService(Context.USB_SERVICE);

		// register someone to listen for "permission granted" dialogs
		PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		usbListener = new InfiniTAMUSBPermissionListener();
		registerReceiver(usbListener, filter);

		// get a ist of USB devices
		HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
		Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();

		// find the depth camera
		UsbDevice depthCamera = null;
		while (deviceIterator.hasNext()) {
			UsbDevice device = deviceIterator.next();

			if (((device.getVendorId() == 0x1d27)&&(device.getProductId() == 0x0600))||
			    ((device.getVendorId() == 0x1d27)&&(device.getProductId() == 0x0601))) {
				depthCamera = device;
			}
		}
		if (depthCamera == null) {
			usbListener = null;
			return false;
		}

		if (manager.hasPermission(depthCamera)) {
			usbListener.setDeviceWithPermission(depthCamera);
		} else {
			manager.requestPermission(depthCamera, permissionIntent);
		}

		return true;
	}

	int waitForUSBPermission()
	{
		UsbManager manager = (UsbManager)getSystemService(Context.USB_SERVICE);

		if (usbListener == null) return -1;
		while (!usbListener.permissionGranted()) {
			SystemClock.sleep(100);
		}

		UsbDevice depthCamera = usbListener.getDevice();
		UsbDeviceConnection con = manager.openDevice(depthCamera);
		return con.getFileDescriptor();
	}

	public static native void InitializeNativeApp(String libdir);
}

class InfiniTAMUSBPermissionListener extends BroadcastReceiver {
	private UsbDevice device;
	private boolean granted;

	public InfiniTAMUSBPermissionListener() {
		granted = false;
		device = null;
	}

	@Override
	public void onReceive(Context context, Intent intent) {
		String action = intent.getAction();
		if (!InfiniTAMApplication.ACTION_USB_PERMISSION.equals(action)) return;

		synchronized (this) {
			UsbDevice tmp_device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

			if (!intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) return;
			if (tmp_device == null) return;

			device = tmp_device;
			granted = true;
		}
	}

	public void setDeviceWithPermission(UsbDevice tmp_device)
	{ device = tmp_device; granted = true; }

	public boolean permissionGranted()
	{ return granted; }

	public UsbDevice getDevice()
	{ return device; }
}

