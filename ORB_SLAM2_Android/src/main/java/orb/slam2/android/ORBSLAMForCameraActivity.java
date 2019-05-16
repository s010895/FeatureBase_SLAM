package orb.slam2.android;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import orb.slam2.android.nativefunc.OrbNdkHelper;


import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.TotalCaptureResult;
import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.text.TextUtils;
import android.util.Log;
import android.util.Size;
import android.view.Menu;
import android.view.MenuItem;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.HandlerThread;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * ORB Test Activity For DataSetMode
 * 
 * @author buptzhaofang@163.com Mar 24, 2016 4:13:32 PM
 *
 */
public class ORBSLAMForCameraActivity extends Activity {
	
	private static final String TAG = "ORB_SLAM:Camera";
	ImageView imgDealed;
	private String externalStorageDirectory = Environment.getExternalStorageDirectory().getPath();

	private static final int REQUEST_CAMERA_PERMISSION = 200;
	private CameraDevice mCameraDevice;
	private ImageReader mImageReader;
	private boolean mSaveImage;
	private CaptureRequest.Builder mCaptureRequestBuilder;
	private CameraCaptureSession mCameraCaptureSession;
	private String cameraId;
	private boolean stopRequestImageThread = false;
	private int CameraImageWidth;
	private int CameraImageHeight;
	private int CameraImageHalfWidth;
	private int CameraImageSize;
	private int img_count;
	private byte[] imageByteArray;
	private Image cameraImage;
	private Handler mBackgroundHandler;
	private HandlerThread mBackgroundThread;
	private Surface surface;
	protected CameraCaptureSession cameraCaptureSessions;
	private TextureView textureView;
	LinearLayout linear;
	private Size mImageDimension;
	String vocPath, calibrationPath;
	private TextView mStatus;
	private TextView x_pose;
	private TextView y_pose;
	private TextView z_pose;
	private TextView yaw;
	private TextView pitch;
	private TextView roll;
	private static final int INIT_FINISHED=0x00010001;
	private static final int INSERT_MSG = 3;
	double lastTime;
	double fps;
	double final_fps;
	double average_fps;
	double counter;
	private float[] pos;
	private boolean              mIsJavaCamera = true;
	private MenuItem             mItemSwitchCamera = null;
	private int cameraCounter;
	private final int CONTEXT_CLIENT_VERSION = 3;

	
	long addr;
	int w,h;
	boolean isSLAMRunning=true;


	
	static {
		System.loadLibrary("ORB_SLAM2_EXCUTOR");
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
				WindowManager.LayoutParams.FLAG_FULLSCREEN);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		setContentView(R.layout.activity_camera_orb);

		//textureView = (TextureView) findViewById(R.id.img_dealed);
		imgDealed = (ImageView) findViewById(R.id.img_dealed);


		counter = 0;
		average_fps = 0;
		img_count = 1;
		cameraCounter = 0;
		vocPath = getIntent().getStringExtra("voc");
		calibrationPath = getIntent().getStringExtra("calibration");
		mSaveImage = getIntent().getBooleanExtra("saveImage",false);
		openCamera();
		CameraImageWidth = mImageDimension.getWidth();
		CameraImageHeight = mImageDimension.getHeight();
		CameraImageHalfWidth = CameraImageWidth / 2;
		CameraImageSize = CameraImageWidth * CameraImageHeight;
		mImageReader = ImageReader.newInstance(CameraImageWidth, CameraImageHeight, ImageFormat.YUV_420_888, 2);
		imageByteArray = new byte[CameraImageSize];
		Log.d(TAG, "init success");
		if (TextUtils.isEmpty(vocPath) || TextUtils.isEmpty(calibrationPath)) {
			Toast.makeText(this, "null param,return!", Toast.LENGTH_LONG)
					.show();
			finish();
		} else {
			Toast.makeText(ORBSLAMForCameraActivity.this, "init has been started!",
					Toast.LENGTH_LONG).show();
			new Thread(new Runnable() {

				@Override
				public void run() {
					// TODO Auto-generated method stub
					OrbNdkHelper.initSystemWithParameters(vocPath,
							calibrationPath);
							Log.e("information==========>",
									"init has been finished!");
							myHandler.sendEmptyMessage(INIT_FINISHED);
							if(!mSaveImage)
								posHandler.sendEmptyMessage(INSERT_MSG);
				}
			}).start();
		}
	}
	Handler posHandler = new Handler()
	{
		@Override
		public void handleMessage(Message msg){
			switch(msg.what)
			{
				case INSERT_MSG:
					new Thread(new Runnable() {
						@Override
						public void run() {

							while(true)
							{


								lastTime = System.nanoTime();
								int[] resultInt=OrbNdkHelper.trackRealTime();
								fps = 1000000000.0 / (System.nanoTime() - lastTime);
								if(counter>0)
								{
									average_fps+=fps;
									final_fps=average_fps/counter;

								}

								resultImg = Bitmap.createBitmap(640, 420,Config.RGB_565);
								//final Bitmap bitmapLeft = Bitmap.createBitmap(CameraImageHalfWidth, CameraImageHeight, Config.ARGB_8888);
								//bitmapLeft.copyPixelsFromBuffer(ByteBuffer.wrap(resultInt));
								resultImg.setPixels(resultInt, 0, 640, 0, 0,640,420);
								//bitmapLeft.setPixels(resultInt, 0, 640, 0, 0,640,400);
								/*
								try{
									OutputStream streamLeft = new FileOutputStream(externalStorageDirectory +
											"/camera2_png/left/Image_" + String.format(Locale.getDefault(), "%07d", img_count) + ".png");
									bitmapLeft.compress(Bitmap.CompressFormat.PNG, 100, streamLeft);
									streamLeft.close();
									Log.w(TAG, "Image saved");
								}catch (Exception e) {
									Log.w(TAG, "SlamThread run, error in save raw img, e: " + e + ", " + img_count);

								}
								*/
								counter++;
								runOnUiThread(new Runnable() {
									@Override
									public void run() {
										// TODO Auto-generated method stub
										imgDealed.setImageBitmap(resultImg);
										//Bitmap bmp = BitmapFactory.decodeResource(getResources(),R.id.img_dealed);


										mStatus.setText("FPS: "+String.format("%.5f",final_fps));

									}
								});

							}
						}
					}).start();
					break;

			}
			super.handleMessage(msg);
		}

	};
	
	Handler myHandler = new Handler() {  
        public void handleMessage(Message msg) {   
             switch (msg.what) {   
                  case INIT_FINISHED:   
                	  Toast.makeText(ORBSLAMForCameraActivity.this,
								"init has been finished!",
								Toast.LENGTH_LONG).show();
          			new Thread(new Runnable() {

        				@Override
        				public void run() {
							while(true){
								do {
									cameraImage = mImageReader.acquireLatestImage();
									//if (cameraImage == null) {
									//Log.d(TAG, "SlamThread run cameraImage == null");
									//}
								} while (cameraImage == null);


								if (cameraImage.getFormat() == ImageFormat.YUV_420_888) {
									//Log.d(TAG, "SlamThread run YUV_420_888");
									Image.Plane[] planes = cameraImage.getPlanes();
									if (planes != null) {
										Image.Plane YPlane = planes[0];
										//Log.d(TAG, "SlamThread run YUV_420_888, pixelStride = " + YPlane.getPixelStride());
										if (YPlane != null) {
											ByteBuffer byteBuffer = YPlane.getBuffer();
											//try {
											//    Log.d(TAG, "SlamThread run length = " + byteBuffer.remaining());
											//    Log.d(TAG, "SlamThread run capacity = " + byteBuffer.capacity());
											//} catch (Exception e) {
											//    Log.w(TAG, "SlamThread run byteBuffer, e: " + e);
											//}
											if (byteBuffer != null) {
												//byteBuffer.rewind();
												byteBuffer.get(imageByteArray);
												cameraImage.close();
												byte[] bitmapData = new byte[4 * CameraImageSize];  // 4 * byteBuffer.capacity()
												int i, j;
												for (i = 0, j = 0; i < CameraImageSize; ++i) {
													bitmapData[j * 4] = bitmapData[j * 4 + 1] = bitmapData[j * 4 + 2] = imageByteArray[i];
													bitmapData[j * 4 + 3] = (byte) 0xff;
													++j;
												}
											if(cameraCounter>5)
											{
												if(!mSaveImage)
												{
													//OrbNdkHelper.trackWithCombinedImage(imageByteArray);
												//final Bitmap bitmap = Bitmap.createBitmap(CameraImageWidth, CameraImageHeight, Bitmap.Config.ARGB_8888);
												//bitmap.copyPixelsFromBuffer(ByteBuffer.wrap(bitmapData));
													//pos = OrbNdkHelper.getCurrentPose();
													runOnUiThread(new Runnable() {
													@Override
													public void run() {
														x_pose.setText("X: "+ String.format("%.5f",pos[0]));
														y_pose.setText("Y: "+ String.format("%.5f",pos[1]));
														z_pose.setText("Z: "+ String.format("%.5f",pos[2]));
														yaw.setText("Yaw: "+ String.format("%.5f",pos[3]));
														pitch.setText("Pitch: "+String.format("%.5f",pos[4]));
														roll.setText("roll: "+String.format("%.5f",pos[5]));
														}
													});
												}
												else{
													try {

														String filenameLeft = externalStorageDirectory + "/camera2_raw/left/rawImage_" +
																String.format(Locale.getDefault(),"%07d", img_count);
														String filenameRight = externalStorageDirectory + "/camera2_raw/right/rawImage_" +
																String.format(Locale.getDefault(), "%07d", img_count);
														FileOutputStream outputStreamLeft = new FileOutputStream(filenameLeft);
														FileOutputStream outputStreamRight = new FileOutputStream(filenameRight);
														//Log.d(TAG, "SlamThread run imageByteArray.length = " + imageByteArray.length);
														for (int k = 0; k < CameraImageHeight; ++k) {
															outputStreamLeft.write(imageByteArray, k * CameraImageWidth, CameraImageHalfWidth);
															outputStreamRight.write(imageByteArray, k * CameraImageWidth + CameraImageHalfWidth, CameraImageHalfWidth);
														}
														outputStreamLeft.close();
														outputStreamRight.close();
														Log.d(TAG, "SlamThread run, succeed in save raw img, img_count: " + img_count);
													/*
														byte[] bitmapDataLeft = new byte[2 * CameraImageSize];  // 4 * byteBuffer.capacity() / 2
														byte[] bitmapDataRight = new byte[2 * CameraImageSize];  // 4 * byteBuffer.capacity() / 2
														for (int k = 0, l = 0, m = 0; k < CameraImageSize; ++k) {
															if ((k % CameraImageWidth) < CameraImageHalfWidth) {
																bitmapDataLeft[l * 4] = bitmapDataLeft[l * 4 + 1] = bitmapDataLeft[l * 4 + 2] = imageByteArray[k];
																bitmapDataLeft[l * 4 + 3] = (byte) 0xff;
																++l;
															} else {
																bitmapDataRight[m * 4] = bitmapDataRight[m * 4 + 1] = bitmapDataRight[m * 4 + 2] = imageByteArray[k];
																bitmapDataRight[m * 4 + 3] = (byte) 0xff;
																++m;
															}
														}
														Bitmap bitmapLeft = Bitmap.createBitmap(CameraImageHalfWidth, CameraImageHeight, Bitmap.Config.ARGB_8888);
														Bitmap bitmapRight = Bitmap.createBitmap(CameraImageHalfWidth, CameraImageHeight, Bitmap.Config.ARGB_8888);
														bitmapLeft.copyPixelsFromBuffer(ByteBuffer.wrap(bitmapDataLeft));
														bitmapRight.copyPixelsFromBuffer(ByteBuffer.wrap(bitmapDataRight));
														OutputStream streamLeft = new FileOutputStream(externalStorageDirectory +
																"/camera2_png/left/Image_" + String.format(Locale.getDefault(), "%07d", img_count) + ".png");
														OutputStream streamRight = new FileOutputStream(externalStorageDirectory +
																"/camera2_png/right/Image_" + String.format(Locale.getDefault(), "%07d", img_count) + ".png");
														bitmapLeft.compress(Bitmap.CompressFormat.PNG, 100, streamLeft);
														bitmapRight.compress(Bitmap.CompressFormat.PNG, 100, streamRight);
														streamLeft.close();
														streamRight.close();
														Log.d(TAG, "SlamThread run, succeed in save png img, img_count: " + img_count);
														*/
														img_count++;
													} catch (Exception e) {
														Log.w(TAG, "SlamThread run, error in save raw img, e: " + e + ", " + img_count);
														img_count = 1;
													}




												}
											  }
											  else
											 {
												cameraCounter++;
												}
											}
										}
									}
								}

							}
						        				}
        			}).start();
                       break;   
             }   
             super.handleMessage(msg);   
        }   
   };

	private Bitmap tmp, resultImg;
	private double timestamp;

	private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
		@Override
		public void onOpened(CameraDevice camera) {
			//This is called when the camera is open
			Log.e(TAG, "onOpened");
			mCameraDevice = camera;
			createCameraPreview();
		}
		@Override
		public void onDisconnected(CameraDevice camera) {
			mCameraDevice.close();
		}
		@Override
		public void onError(CameraDevice camera, int error) {
			mCameraDevice.close();
			mCameraDevice = null;
		}
	};
	private void openCamera() {
		CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
		Log.e(TAG, "is camera open");
		try {
			cameraId = manager.getCameraIdList()[0];
			CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
			StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
			assert map != null;
			mImageDimension = map.getOutputSizes(SurfaceTexture.class)[0];
			// Add permission for camera and let user grant the permission
			if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
				ActivityCompat.requestPermissions(ORBSLAMForCameraActivity.this, new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE}, REQUEST_CAMERA_PERMISSION);
				return;
			}
			manager.openCamera(cameraId, stateCallback, null);
		} catch (CameraAccessException e) {
			e.printStackTrace();
		}
		Log.e(TAG, "openCamera X");
	}
	protected void createCameraPreview() {
		try {

			mCaptureRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
			mCaptureRequestBuilder.addTarget(mImageReader.getSurface());
			mCaptureRequestBuilder.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
			/*
			mCaptureRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_AUTO);
            mCaptureRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_AUTO);
            mCaptureRequestBuilder.set(CaptureRequest.SHADING_MODE, CaptureRequest.SHADING_MODE_AUTO);
            mCaptureRequestBuilder.set(CaptureRequest.FLASH_MODE, CaptureRequest.FLASH_MODE_AUTO);
            mCaptureRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_AUTO);
            //mCaptureRequestBuilder.set(CaptureRequest.SENSOR_EXPOSURE_TIME, Long.valueOf("1000000")); // Units: Nanoseconds
            mCaptureRequestBuilder.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_AUTO);
            mCaptureRequestBuilder.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_AUTO);
			*/
			mCameraDevice.createCaptureSession(Arrays.asList(mImageReader.getSurface()), new CameraCaptureSession.StateCallback(){
				@Override
				public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
					//The camera is already closed
					if (null == mCameraDevice) {
						return;
					}
					// When the session is ready, we start displaying the preview.
					cameraCaptureSessions = cameraCaptureSession;
					updatePreview();
				}
				@Override
				public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
					Toast.makeText(ORBSLAMForCameraActivity.this, "Configuration change", Toast.LENGTH_SHORT).show();
				}
			}, null);
		} catch (CameraAccessException e) {
			e.printStackTrace();
		}
	}
	final CameraCaptureSession.CaptureCallback captureListener= new CameraCaptureSession.CaptureCallback() {
		@Override
		public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result) {
			super.onCaptureCompleted(session, request, result);
			//Toast.makeText(AndroidCameraApi.this, "Saved:" + file, Toast.LENGTH_SHORT).show();
			try {
				session.capture(mCaptureRequestBuilder.build(), captureListener, mBackgroundHandler);
			} catch (CameraAccessException e) {
				e.printStackTrace();
			}
		}
	};
	protected void updatePreview() {
		if(null == mCameraDevice) {
			Log.e(TAG, "updatePreview error, return");
		}
		//mCaptureRequestBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
		try {
			cameraCaptureSessions.setRepeatingRequest(mCaptureRequestBuilder.build(),captureListener , mBackgroundHandler);
			//cameraCaptureSessions.setRepeatingBurst(Arrays.asList(mCaptureRequestBuilder.build()), null, mBackgroundHandler);
		} catch (CameraAccessException e) {
			e.printStackTrace();
		}
	}

	protected void startBackgroundThread() {
		mBackgroundThread = new HandlerThread("Camera Background");
		mBackgroundThread.start();
		mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
		startBackgroundThread();

		

	}

	
	
	@Override
	protected void onStart() {
		// TODO Auto-generated method stub
		super.onStart();
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();


	}
	
	public void onDestroy() {
        super.onDestroy();

    }
	@Override
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.i(TAG, "called onCreateOptionsMenu");
        mItemSwitchCamera = menu.add("stop SLAM");
        return true;
    }

	 @Override
	    public boolean onOptionsItemSelected(MenuItem item) {
	        String toastMesage = new String();
	        Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);

	        if (item == mItemSwitchCamera) {
	        	isSLAMRunning=false;
//	            mOpenCvCameraView.setVisibility(SurfaceView.GONE);
//	            mIsJavaCamera = !mIsJavaCamera;
//
//	            if (mIsJavaCamera) {
//	                mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
//	                toastMesage = "Java Camera";
//	            } else {
//	                mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_native_surface_view);
//	                toastMesage = "Native Camera";
//	            }
//
//	            mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
//	            mOpenCvCameraView.setCvCameraViewListener(this);
//	            mOpenCvCameraView.enableView();
//	            Toast toast = Toast.makeText(this, toastMesage, Toast.LENGTH_LONG);
//	            toast.show();
	        }

	        return true;
	    }


}
