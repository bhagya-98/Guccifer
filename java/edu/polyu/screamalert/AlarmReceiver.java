package edu.polyu.screamalert;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.PowerManager;


public class AlarmReceiver extends BroadcastReceiver{
	
	private static PowerManager.WakeLock wakeLock = null;
	private static String LOCK_TAG;

	// Wake up system to run the checking service of this application
	// PARTIAL_WAKE_LOCK means that the CPU will continue to run even if the screen is off
	public static synchronized void acquireLock(Context ctx)
	{
		LOCK_TAG = ctx.getPackageName();
		if (wakeLock == null)
		{
			PowerManager mgr = (PowerManager) ctx.getSystemService(Context.POWER_SERVICE);
			wakeLock = mgr.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, LOCK_TAG);
			wakeLock.setReferenceCounted(true);
		}
		wakeLock.acquire();
	}
	
	//Release system resource to allow the system to sleep
	public static synchronized void releaseLock()
	{
		if (wakeLock != null)
			wakeLock.release();
	}
	
	//Start the service by calling SoundProcessingService.class
	@Override
	public void onReceive(Context context, Intent intent) {
		acquireLock(context);
		Intent expireCheckService = new Intent(context, SoundProcessingService.class);
		context.startService(expireCheckService);
	}
	
}
