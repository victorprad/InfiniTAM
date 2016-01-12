// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
import android.util.Log;

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
	boolean stopRequested;

	InfiniTAMProcessor()
	{
		view = null;
		stopRequested = false;
	}

	public void attachVisualisation(InfiniTAMView _view)
	{
		view = _view;
	}

	public void detachVisualisation(InfiniTAMView _view)
	{
		view = null;
	}

	public void requestStop()
	{
		stopRequested = true;
	}

	public void run()
	{
		int usb_fd = InfiniTAMApplication.getApplication().waitForUSBPermission();
		stopRequested = false;
		StartProcessing((usb_fd >= 0)?1:0);

		while (true) {
			int ret = ProcessFrame();
			if (view != null) view.requestRender();
			if (ret == 0) break;
			if (stopRequested) break;
		}
	}

	public static native void StartProcessing(int useLiveCamera);
	public static native int ProcessFrame();
}

