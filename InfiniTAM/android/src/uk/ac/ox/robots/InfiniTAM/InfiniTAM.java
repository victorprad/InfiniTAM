package uk.ac.ox.robots.InfiniTAM;

import android.app.Activity;
import android.os.Bundle;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
//import android.opengl.GL10;
//import android.opengl.EGLConfig;
import java.lang.Thread;

public class InfiniTAM extends Activity
{
	static
	{
		System.loadLibrary("InfiniTAM");
	}

	private InfiniTAMView view;
	private /*InfiniTAMProcessing*/Thread processor;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);

		InitializeNativeApp();

		view = new InfiniTAMView(this);

		// Tell EGL to use a ES 2.0 Context
//		view.setupEGLContextClientVersion(2);

		// Set the renderer
//		view.setRenderer(new InfiniTAM_Renderer());

		setContentView(view);

		processor = new Thread(new InfiniTAMProcessingThread(view));
		processor.start();
	}

	@Override
	protected void onPause() 
	{
		super.onPause();
		view.onPause();
//		processor.pause();
	}

	@Override
	protected void onResume() 
	{
		super.onResume();
		view.onResume();
//		processor.resume();
	}

	public static native void InitializeNativeApp();
}

class InfiniTAMView extends GLSurfaceView
{

	private final InfiniTAMRenderer mRenderer;

	public InfiniTAMView(Context context) {
		super(context);

		// Create an OpenGL ES 2.0 context.
		setEGLContextClientVersion(1);

		// Set the Renderer for drawing on the GLSurfaceView
		mRenderer = new InfiniTAMRenderer();
		setRenderer(mRenderer);

		// Render the view only when there is a change in the drawing data
		setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
	}
}

class InfiniTAMRenderer implements GLSurfaceView.Renderer
{
	public void onDrawFrame(GL10 gl) 
	{
		//gl.glClear(gl.GL_COLOR_BUFFER_BIT);
		RenderGL();
	}

	public void onSurfaceChanged(GL10 gl, int width, int height) 
	{		
		//gl.glViewport(0, 0, width, height);
		ResizeGL(width, height);
	}

	public void onSurfaceCreated(GL10 gl, EGLConfig config) 
	{
		//gl.glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
		InitGL();
	}

	public static native void InitGL();
	public static native void ResizeGL(int newWidth, int newHeight);
	public static native void RenderGL();
}

class InfiniTAMProcessingThread implements Runnable
{
	private InfiniTAMView view;

	InfiniTAMProcessingThread(InfiniTAMView _view)
	{
		view = _view;
	}

	public void run()
	{
		while (true) {
			int ret = ProcessFrame();
			view.requestRender();
			if (ret == 0) break;
		}
	}

	public static native int ProcessFrame();
}

