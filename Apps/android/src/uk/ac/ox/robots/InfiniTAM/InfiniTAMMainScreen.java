// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.app.Activity;
import android.os.Bundle;
import android.content.Context;
import android.opengl.GLSurfaceView;

import android.view.View;
import android.view.Gravity;
import android.widget.Button;
import android.widget.TextView;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import uk.ac.ox.robots.InfiniTAM.InfiniTAMApplication;
import uk.ac.ox.robots.InfiniTAM.InfiniTAMProcessor;

public class InfiniTAMMainScreen extends Activity
{
	private InfiniTAMGLView mGLView;
	private UpdatableTextView mFPSDisplay;

	private Thread processingThread;
	private InfiniTAMProcessor processor;
	private IMUReceiver imuReceiver;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);

		View v = setupView();
		processor = new InfiniTAMProcessor();

		// Tell EGL to use a ES 2.0 Context
//		view.setupEGLContextClientVersion(2);

		// Set the renderer
//		view.setRenderer(new InfiniTAM_Renderer());

		processingThread = new Thread(processor);
		processingThread.start();
		setContentView(v);
	}

	private View setupView()
	{
		FrameLayout mainLayout = new FrameLayout(this);

		mGLView = new InfiniTAMGLView(this);
		mainLayout.addView(mGLView);

		LinearLayout rowOfButtons = new LinearLayout(this);
		rowOfButtons.setOrientation(LinearLayout.VERTICAL);
		{
			TextView title = new TextView(this);
			title.setText("InfiniTAM, Android version");
			rowOfButtons.addView(title);

			TextView fpsDisplay = new TextView(this);
			fpsDisplay.setText("FPS currently unknown");
			rowOfButtons.addView(fpsDisplay);

			mFPSDisplay = new UpdatableTextView(this, fpsDisplay);
		}
		{
			Button button = new Button(this);
			button.setText("Record Video");
			button.setOnClickListener(new View.OnClickListener() {
				public void onClick(View v) {
					getProcessor().toggleRecordingMode();
				}
			});
			rowOfButtons.addView(button);
		}
		{
			Button button = new Button(this);
			button.setText("Exit");
			button.setOnClickListener(new View.OnClickListener() {
				public void onClick(View v) {
					// Perform action on click
					getProcessor().requestStop();
					try { processingThread.join(); } catch (Exception e) {}
					finish();
				}
			});
			rowOfButtons.addView(button);
		}
		mainLayout.addView(rowOfButtons, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.MATCH_PARENT, Gravity.RIGHT));

		return mainLayout;
	}

	@Override
	protected void onResume() 
	{
		super.onResume();
		mGLView.onResume();
		processor.attachVisualisation(mGLView, mFPSDisplay);
//		processor.resume();
	}

	@Override
	protected void onPause() 
	{
		processor.detachVisualisation(mGLView, mFPSDisplay);
		super.onPause();
		mGLView.onPause();
//		processor.pause();
	}

	@Override
	protected void onDestroy()
	{
		super.onDestroy();
	}

	public InfiniTAMProcessor getProcessor()
	{
		return processor;
	}
}

class InfiniTAMGLView extends GLSurfaceView
{
	private final InfiniTAMRenderer mRenderer;

	public InfiniTAMGLView(Context context) {
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

class UpdatableTextView
{
	private Context mContext;
	private TextView mTextView;

	public UpdatableTextView(Context context, TextView textView)
	{
		mContext = context;
		mTextView = textView;
	}

	public void setText(final CharSequence text)
	{
		((Activity) mContext).runOnUiThread(new Runnable() {
		        @Override
		        public void run() {
				(mTextView).setText(text);
			}
		});
	}
}
