package uk.ac.ox.robots.InfiniTAM;

import android.app.Activity;
import android.os.Bundle;
import android.content.Context;
import android.opengl.GLSurfaceView;
import uk.ac.ox.robots.InfiniTAM.InfiniTAMApplication;
import uk.ac.ox.robots.InfiniTAM.InfiniTAMProcessor;

public class InfiniTAMMainScreen extends Activity
{
	private InfiniTAMView view;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);

		view = new InfiniTAMView(this);
		InfiniTAMApplication.getApplication().getProcessor().attachVisualisation(view);

		// Tell EGL to use a ES 2.0 Context
//		view.setupEGLContextClientVersion(2);

		// Set the renderer
//		view.setRenderer(new InfiniTAM_Renderer());

		setContentView(view);
	}

	@Override
	protected void onPause() 
	{
		InfiniTAMApplication.getApplication().getProcessor().detachVisualisation(view);
		super.onPause();
		view.onPause();
//		processor.pause();
	}

	@Override
	protected void onResume() 
	{
		super.onResume();
		view.onResume();
		InfiniTAMApplication.getApplication().getProcessor().attachVisualisation(view);
//		processor.resume();
	}

	@Override
	protected void onStop()
	{
		InfiniTAMApplication.getApplication().getProcessor().detachVisualisation(view);
		super.onStop();
	}

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

