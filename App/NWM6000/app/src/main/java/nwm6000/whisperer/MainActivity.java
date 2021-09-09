package nwm6000.whisperer;

import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    TextView yes;
    public MainActivity() {
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        senSensorManager.registerListener(this, senAccelerometer , SensorManager.SENSOR_DELAY_NORMAL);
        yes =findViewById(R.id.yes);

        findBT();
        createBroadcastReceiver();
        beginListenForData();
    }


    private void openBTWithChecks() {
        if(mmDevice!=null){
            try {
                boolean good = openBT();
                if(!good){
                    Log.i("HH", "bluetooth is off");
                    if(mBluetoothAdapter.isEnabled())
                        mBluetoothAdapter.startDiscovery();
                }
            } catch (IOException e) {
                Log.i("HH", "e " + e.toString());
                if(mBluetoothAdapter.isEnabled())
                    mBluetoothAdapter.startDiscovery();
            }
        } else {
            Log.i("HH", "mmdevice is null");
            if(mBluetoothAdapter.isEnabled())
                mBluetoothAdapter.startDiscovery();
        }
    }

    private boolean connected = false;
    private void createBroadcastReceiver() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothDevice.ACTION_FOUND);
        filter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        filter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);

        registerReceiver(receiver, filter);
    }

    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            assert action != null;
            switch (action) {
                case BluetoothDevice.ACTION_FOUND:
                    BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    if (device != null) {
                        String deviceName = device.getName();
                        /*String deviceMAC = device.getAddress();*/
                        if(deviceName!=null){
                            if(deviceName.equals("HC-05")){
                                Log.i("HHH", "got the device man");
                            }
                        }
                    }
                    break;
                case BluetoothDevice.ACTION_ACL_CONNECTED:
                    Log.i("HHH", "connected");
                    connected = true;
                    break;
                case BluetoothDevice.ACTION_ACL_DISCONNECTED:
                    Log.i("HHH", "disconnected");
                    connected = false;
                    mBluetoothAdapter.startDiscovery();
                    /*stopSelfNo();*/
                    break;
                case BluetoothAdapter.ACTION_STATE_CHANGED:
                    Log.i("HH", "bluetooth aaaa");
                    if (mBluetoothAdapter.getState() == BluetoothAdapter.STATE_ON) {
                        // The user bluetooth is turning off yet, but it is not disabled yet.
                        Log.i("HH", "bluetooth on");
                        mBluetoothAdapter.startDiscovery();

                    } else if (mBluetoothAdapter.getState() == BluetoothAdapter.STATE_OFF) {
                        Log.i("HH", "bluetooth off");
                        // The user bluetooth is already disabled.
                    }
                    break;
            }
        }
    };

    void beginListenForData()  {
        /*final Handler handler = new Handler();*/
        final byte delimiter = 10; //This is the ASCII code for a newline character

        final Handler handler = new Handler();
        stopWorker = false;
        readBufferPosition = 0;
        readBuffer = new byte[1024];
        workerThread = new Thread(new Runnable()
        {
            public void run()
            {
                Log.i("HH", "stopWorker " + stopWorker);
                while(!stopWorker) {
                    if(mmInputStream!=null){
                        try {
                            int bytesAvailable = mmInputStream.available();
                            if(bytesAvailable > 0) {
                                byte[] packetBytes = new byte[bytesAvailable];
                                mmInputStream.read(packetBytes);
                                for(int i=0; i<bytesAvailable; i++) {
                                    byte b = packetBytes[i];
                                    if(b == delimiter) {
                                        byte[] encodedBytes = new byte[readBufferPosition];
                                        System.arraycopy(readBuffer, 0, encodedBytes, 0, encodedBytes.length);
                                        final String data = new String(encodedBytes, "US-ASCII");
                                        readBufferPosition = 0;

                                    handler.post(new Runnable() {
                                        public void run() {
                                        Log.i("HH", "data " + data);
                                        Toast.makeText(context, data, Toast.LENGTH_LONG).show();
                                        //onPostExecuto(data);
                                        }
                                    });
                                    } else {
                                        readBuffer[readBufferPosition++] = b;
                                    }
                                }
                            }
                        } catch (Exception e) {
                            /*if(try_to_reconnect())
                                break;*/
                            Log.i("HH", "reading loop crash " + e.toString());
                        }
                    }
                }
            }
        });

        workerThread.start();
    }

    private Context context;
    private Intent intent;
    BluetoothAdapter mBluetoothAdapter;
    BluetoothSocket mmSocket;
    BluetoothDevice mmDevice;
    OutputStream mmOutputStream;
    InputStream mmInputStream;
    Thread workerThread;
    byte[] readBuffer;
    int readBufferPosition;
    volatile boolean stopWorker;

    void findBT() {
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if(mBluetoothAdapter == null) {
            Log.i("HH", "No bluetooth adapter available");
            return;
        }

        if(!mBluetoothAdapter.isEnabled()) {
            Log.i("HH", "bluetooth is off");
            return;
        }
        Log.i("HH", "start discovery");
        mBluetoothAdapter.startDiscovery();

        Log.i("HH", "done");
        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        Log.i("HH", "pairedDevices.size() " + pairedDevices.size());
        if(pairedDevices.size() > 0) {
            for(BluetoothDevice device : pairedDevices) {
                Log.i("HH", "device " + device.getName());
                if(device.getName().equals("HC-05")) {
                    mmDevice = device;
                    break;
                }
            }
        }
        Log.i("HH", "Bluetooth Device Found");
        openBTWithChecks();
    }


    UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); //Standard SerialPortService ID
    boolean openBT() throws IOException
    {
        if(!mBluetoothAdapter.isEnabled())
        {
            Log.i("HH", "bluetooth is off");
            return false;
        }

        try{
            if(mmSocket!=null){
                mmSocket.close();
            }
        } catch(Exception e){
            Log.i("HH", "failed closing " + e.toString());
        }
        mmSocket = mmDevice.createRfcommSocketToServiceRecord(uuid);
        mmSocket.connect();
        Log.i("HH", "mmOutputStream bef " + mmOutputStream);
        mmOutputStream = mmSocket.getOutputStream();
        Log.i("HH", "mmOutputStream  afer" + mmOutputStream);
        mmInputStream = mmSocket.getInputStream();

        Log.i("HH", "Bluetooth Opened");
        return true;
    }

    protected void onPause() {
        super.onPause();
        senSensorManager.unregisterListener(this);
    }
    protected void onResume() {
        super.onResume();
        senSensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    private SensorManager senSensorManager;
    private Sensor senAccelerometer;

    @Override
    public void onSensorChanged(SensorEvent event) {
        Sensor mySensor = event.sensor;

        if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];
            yes.setText("x " + x + "\ny " + y + "\nz " + z);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void spamClicked(View view) {
        try {
            Log.i("HH", "mmOutputStream " + mmOutputStream);
            mmOutputStream.write("0\n".getBytes());
        } catch (Exception e) {
            Log.i("HH", "lol" + e.toString());
        }
    }
}
