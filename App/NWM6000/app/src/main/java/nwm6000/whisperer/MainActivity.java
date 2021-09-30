package nwm6000.whisperer;

import androidx.appcompat.app.AppCompatActivity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import com.jem.rubberpicker.RubberSeekBar;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    List<TextView> value_elements = new ArrayList<>();
    TextView phone_gyro_preview, connected_status, connect, console_replies, requestValues, send_values;
    List<Float> values_to_send = new ArrayList<>();
    LinearLayout settings;
    int additional_parameters = 2, balance_track_delta_on_each_side = 5;
    List<RubberSeekBar> track_elements = new ArrayList<>();
    List<Float> mins = new ArrayList<>(), maxes = new ArrayList<>();
    private boolean settings_open = false;
    private boolean connected = false;
    private boolean running = false;
    final Handler handler2 = new Handler();
    final byte delimiter = 10; //This is the ASCII code for a newline character

    private Context context;
    BluetoothAdapter mBluetoothAdapter;
    BluetoothSocket mmSocket;
    BluetoothDevice mmDevice;
    OutputStream mmOutputStream;
    InputStream mmInputStream;
    Thread workerThread;
    byte[] readBuffer;
    int readBufferPosition;

    UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); //Standard SerialPortService ID
    private SensorManager senSensorManager;
    private Sensor senAccelerometer;
    private float x=(float) 0.0, y=(float) 0.0, z=(float) 0.0;
    private float initial_x=(float) 0.0, initial_y=(float) 0.0, initial_z=(float) 0.0;

    private boolean gyro_stream_on = false;
    private boolean requested_values = false;
    private boolean state_of_connect_button = false;
    private final int min_max_deltas = 20;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        context = this;
        Thread.setDefaultUncaughtExceptionHandler(new TopExceptionHandler());
        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        senSensorManager.registerListener(this, senAccelerometer , SensorManager.SENSOR_DELAY_NORMAL);
        phone_gyro_preview =findViewById(R.id.phone_gyro_preview);
        connected_status =findViewById(R.id.connected_status);
        requestValues =findViewById(R.id.requestValues);
        connect =findViewById(R.id.connect);
        send_values =findViewById(R.id.send_values);
        console_replies =findViewById(R.id.console_replies);
        settings = findViewById(R.id.settings);

        connectedStatus();
        createBroadcastReceiver();

        final SharedPreferences prefs = this.getSharedPreferences("nwm6000.whisperer", Context.MODE_PRIVATE);
        final SharedPreferences.Editor editor = prefs.edit();

        value_elements.add((TextView) findViewById(R.id.P1_value));
        value_elements.add((TextView) findViewById(R.id.i1_value));
        value_elements.add((TextView) findViewById(R.id.d1_value));
        value_elements.add((TextView) findViewById(R.id.P2_value));
        value_elements.add((TextView) findViewById(R.id.i2_value));
        value_elements.add((TextView) findViewById(R.id.d2_value));
        value_elements.add((TextView) findViewById(R.id.switch_value));
        value_elements.add((TextView) findViewById(R.id.balance_value));

        track_elements = new ArrayList<>();
        track_elements.add((RubberSeekBar) findViewById(R.id.P1_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.i1_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.d1_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.P2_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.i2_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.d2_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.switch_track));
        track_elements.add((RubberSeekBar) findViewById(R.id.balance_track));

        final List<Float> defaults = new ArrayList<>();
        defaults.add((float) 30.0);
        defaults.add((float) 280);
        defaults.add((float) 0.8);
        defaults.add((float) 135);
        defaults.add((float) 525);
        defaults.add((float) 9.1);
        defaults.add((float) 8);
        defaults.add((float) 173.85);

        // only applies defaults to sharedpreferences if this is first time launch
        float dummy_value = prefs.getFloat("slider 0", -1);

        if(dummy_value==-1){
            for(int z=0; z<defaults.size()-additional_parameters; z++){
                // save value in sharedPreferences
                editor.putFloat("slider " + z, defaults.get(z)).apply();
            }
            editor.putFloat("switch", defaults.get(defaults.size()-2)).apply();
            editor.putFloat("balance", defaults.get(defaults.size()-1)).apply();
        }

        // Load values from sharedpreferences
        values_to_send = new ArrayList<>();
        for(int j=0; j<defaults.size()-additional_parameters; j++){
            values_to_send.add(prefs.getFloat("slider " + j, defaults.get(j)));
        }
        values_to_send.add(prefs.getFloat("switch", defaults.get(defaults.size()-2)));
        values_to_send.add(prefs.getFloat("balance", defaults.get(defaults.size()-1)));

        mins.add((float) 0);
        mins.add((float) 0);
        mins.add((float) 0);
        mins.add((float) 0);
        mins.add((float) 0);
        mins.add((float) 0);
        float min = values_to_send.get(values_to_send.size()-2) - min_max_deltas;
        mins.add(min < 0? 0 : min);
        mins.add((float) (values_to_send.get(values_to_send.size()-1) - balance_track_delta_on_each_side));

        maxes.add((float) 200);
        maxes.add((float) 2000);
        maxes.add((float) 5);
        maxes.add((float) 200);
        maxes.add((float) 2000);
        maxes.add((float) 20);
        float max = values_to_send.get(values_to_send.size()-2) + min_max_deltas;
        maxes.add(max);
        maxes.add((float) (values_to_send.get(values_to_send.size()-1) + balance_track_delta_on_each_side));

        // define a starting naming fr each
        final List<String> strings = new ArrayList<>();
        strings.add("Proportional 1");
        strings.add("Integral 1");
        strings.add("Derivative 1");
        strings.add("Proportional 2");
        strings.add("Integral 2");
        strings.add("Derivative 2");
        strings.add("Switch");
        strings.add("Balance");


        for(int j=0; j<track_elements.size(); j++){
            final int finalJ = j;

            // don't apply zero to the track bar of the balance since it has a value alrdy (balance above)
            if(values_to_send.get(j) > maxes.get(j))
                values_to_send.set(j, maxes.get(j));
            else if(values_to_send.get(j) < mins.get(j))
                values_to_send.set(j, mins.get(j));

            int value_in_percent = (int) (( values_to_send.get(j) - mins.get(j) ) * 100 / (maxes.get(j)-mins.get(j)));
            if(value_in_percent>100)
                value_in_percent = 100;
            else if(value_in_percent<0)
                value_in_percent = 0;
            track_elements.get(j).setCurrentValue(value_in_percent);
            value_elements.get(j).setText(String.valueOf(values_to_send.get(j)));

            track_elements.get(j).setOnRubberSeekBarChangeListener(new RubberSeekBar.OnRubberSeekBarChangeListener() {
                @Override
                public void onProgressChanged(RubberSeekBar rubberSeekBar, int i, boolean b) {

                    // if the change was applied by me
                    if(b){
                        values_to_send.set(finalJ, mins.get(finalJ) + ((float) (maxes.get(finalJ)-mins.get(finalJ)))*i/100);
                        value_elements.get(finalJ).setText(String.valueOf(values_to_send.get(finalJ)));

                        // display values in logcat temporarily
                        //Log.i("HH", strings.get(finalJ) + " has value " + values_to_send.get(finalJ));
                        //print(strings.get(finalJ) + " has value " + values_to_send.get(finalJ));
                    }
                }

                @Override public void onStartTrackingTouch(RubberSeekBar rubberSeekBar) {}
                @Override public void onStopTrackingTouch(RubberSeekBar rubberSeekBar) {

                    // send updated value to arduino
                    if(finalJ==defaults.size()-1){
                        // save value in sharedPreferences
                        editor.putFloat("balance", values_to_send.get(finalJ)).apply();
                        sendCommand("B " + values_to_send.get(finalJ));
                    } else if(finalJ==defaults.size()-2){
                        // save value in sharedPreferences
                        editor.putFloat("switch", values_to_send.get(finalJ)).apply();
                        sendCommand("S " + values_to_send.get(finalJ));
                    } else {
                        // save value in sharedPreferences
                        editor.putFloat("slider " + finalJ, values_to_send.get(finalJ)).apply();
                        sendCommand("PID " + finalJ + " " + values_to_send.get(finalJ));
                    }
                }
            });
        }
    }

    @Override
    public void onBackPressed() {
        if(settings_open){
            SettingsPage(false);
        } else {
            super.onBackPressed();
        }
    }

    private void SettingsPage(boolean show) {
        if(show){
            settings.setVisibility(View.VISIBLE);
            connect.setVisibility(View.INVISIBLE);
            requestValues.setVisibility(View.INVISIBLE);
            send_values.setVisibility(View.INVISIBLE);
            console_replies.setVisibility(View.INVISIBLE);
        } else {
            settings.setVisibility(View.GONE);
            connect.setVisibility(View.VISIBLE);
            requestValues.setVisibility(View.VISIBLE);
            send_values.setVisibility(View.VISIBLE);
            console_replies.setVisibility(View.VISIBLE);
        }
        settings_open = show;
    }

    void print(Object log){
        Toast.makeText(context, String.valueOf(log), Toast.LENGTH_LONG).show();
    }

    private void openBTWithChecks() {
        if(mmDevice!=null){
            try {
                boolean good = openBT();
                if(!good){
                    //if(mBluetoothAdapter.isEnabled())
                        //mBluetoothAdapter.startDiscovery();
                }
            } catch (IOException e) {
                Log.i("HH", "e " + e.toString());
                //if(mBluetoothAdapter.isEnabled())
                    //mBluetoothAdapter.startDiscovery();
            }
        } else {
            Log.i("HH", "mmdevice is null");
            //if(mBluetoothAdapter.isEnabled())
                //mBluetoothAdapter.startDiscovery();
        }
    }

    private void createBroadcastReceiver() {
        IntentFilter filter = new IntentFilter();
        //filter.addAction(BluetoothDevice.ACTION_FOUND);
        filter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        //filter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);

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
                        if(mmDevice==null){
                            if(deviceName!=null){
                                if(deviceName.equals("HC-05")){
                                    Log.i("HHH", "Found the devicec");
                                    //findBT();
                                }
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
                    if(mBluetoothAdapter==null)
                        return;
                    //mBluetoothAdapter.startDiscovery();
                    /*stopSelfNo();*/
                    break;
                case BluetoothAdapter.ACTION_STATE_CHANGED:
                    Log.i("HH", "bluetooth ACTION_STATE_CHANGED called mate");
                    if(mBluetoothAdapter==null)
                        return;

                    if (mBluetoothAdapter.getState() == BluetoothAdapter.STATE_ON) {
                        // The user bluetooth is turning off yet, but it is not disabled yet.
                        Log.i("HH", "bluetooth on");
                        //mBluetoothAdapter.startDiscovery();

                    } else if (mBluetoothAdapter.getState() == BluetoothAdapter.STATE_OFF) {
                        Log.i("HH", "bluetooth off");
                        // The user bluetooth is already disabled.
                    }
                    break;
            }
        }
    };

    void beginListenForData()  {

        readBufferPosition = 0;
        readBuffer = new byte[1024];
        workerThread = new Thread(new Runnable(){
            public void run(){
                running = true;
                while(running) {

                    if(mmDevice==null){
                        Log.i("HH", "mmDevice is null");
                        break;
                    }

                    try {
                        int bytesAvailable = mmInputStream.available();
                        connected = true;
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

                                handler2.post(new Runnable() {
                                    public void run() {
                                    //Log.i("HH", "data " + data);
                                    console_replies.setText(data);
                                    treat_robottu_msg(data);
                                    //onPostExecuto(data);
                                    }
                                });
                                } else {
                                    readBuffer[readBufferPosition++] = b;
                                }
                            }
                        }
                    } catch (IOException e) {
                        running = false;
                        /*if(try_to_reconnect())
                            break;*/
                        requested_values = false;
                        Log.i("HH", "reading loop crash " + e.toString());

                        mBluetoothAdapter = null;
                        mmSocket = null;
                        mmDevice = null;
                        mmOutputStream = null;
                        mmInputStream = null;

                        handler2.post(new Runnable() {
                            public void run() {
                                connected_status.setText("Failed: Socket");
                                connected_status.setTextColor(getResources().getColor(R.color.failed));
                                //onPostExecuto(data);
                            }
                        });
                    } catch (Exception e2) {
                        running = false;
                        requested_values = false;
                        Log.i("HH", "reading loop crash " + e2.toString());

                        handler2.post(new Runnable() {
                            public void run() {
                                connected_status.setText("Failed: Exception");
                                connected_status.setTextColor(getResources().getColor(R.color.failed));
                                //onPostExecuto(data);
                            }
                        });
                    }
                }
                connected = false;

            }
        });

        workerThread.start();
    }

    private void treat_robottu_msg(String data) {
        Log.i("HH", "data " + data);
        if(data.isEmpty())
            return;

        String[] parts = data.split(" ");

        switch(parts[0]){
            case "VALUES":
                int track_elements_size = track_elements.size();
                for(int i=0; i<track_elements_size-additional_parameters; i++){
                    float value = (float) Double.parseDouble(parts[i+1]);
                    int value_in_percent = percent_from_value(i, value);
                    values_to_send.set(i, value);
                    track_elements.get(i).setCurrentValue(value_in_percent);
                    value_elements.get(i).setText(String.valueOf(values_to_send.get(i)));
                }

                float switch_value = (float) Double.parseDouble(parts[track_elements_size+1-2]);
                int switch_value_in_percent = percent_from_value(track_elements_size-2, switch_value);
                values_to_send.set(track_elements_size-2, switch_value);
                track_elements.get(track_elements_size-2).setCurrentValue(switch_value_in_percent);
                value_elements.get(track_elements_size-2).setText(String.valueOf(values_to_send.get(track_elements_size-2)));

                float balance_value = (float) Double.parseDouble(parts[track_elements_size+1-1]);
                int balance_value_in_percent = percent_from_value(track_elements_size-1, balance_value);
                values_to_send.set(track_elements_size-1, balance_value);
                track_elements.get(track_elements_size-1).setCurrentValue(balance_value_in_percent);
                value_elements.get(track_elements_size-1).setText(String.valueOf(values_to_send.get(track_elements_size-1)));

                updateRangesClicked(null);
                break;
            default:
                console_replies.setText("Command" + parts[0] + " unknown");
                break;
        }

    }

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
        //Log.i("HH", "start discovery");
        //mBluetoothAdapter.startDiscovery();

        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if(pairedDevices.size() > 0) {
            for(BluetoothDevice device : pairedDevices) {
                Log.i("HH", "device " + device.getName());
                if(device.getName().equals("HC-05")) {
                    mmDevice = device;
                    openBTWithChecks();
                    break;
                }
            }
        }
    }

    boolean openBT() throws IOException{


        if(!mBluetoothAdapter.isEnabled()){
            Log.i("HH", "bluetooth is off");
            return false;
        }

        try{
            if(mmSocket!=null){
                mmSocket.close();
                Log.i("HH", "closed socket");
                mmSocket = null;
            }
        } catch(Exception e){
            Log.i("HH", "failed closing " + e.toString());
        }

        if(mmDevice==null){
            Log.i("HH", "device is null goign back to start");
            findBT();
            return false;
        }

        if(mmSocket==null)
            mmSocket = mmDevice.createRfcommSocketToServiceRecord(uuid);
        mmSocket.connect();
        Log.i("HH", "connected socket");
        mmOutputStream = mmSocket.getOutputStream();
        mmInputStream = mmSocket.getInputStream();

        Log.i("HH", "Bluetooth Opened");
        beginListenForData();
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

    @Override
    public void onSensorChanged(SensorEvent event) {
        Sensor mySensor = event.sensor;

        if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            x = event.values[0];
            y = event.values[1];
            z = event.values[2];
            phone_gyro_preview.setText("x " + x + "\ny " + y + "\nz " + z);

            if(gyro_stream_on){

                int ys = Math.round(y - initial_y);
                int zs  = Math.round(z  - initial_z);

                if(ys!=previous_y || zs!=previous_z)
                    sendCommand("D " + ys + " " + zs);

                previous_y = ys;
                previous_z  = zs;

            }
        }
    }
    private int previous_y, previous_z;

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private void sendCommand(String command) {
        try {
            String to_send = command + "\n";
            Log.i("HH", "SENDCOMMAND Sending command=" + to_send);
            mmOutputStream.write(to_send.getBytes());
        } catch (Exception e) {
            e.printStackTrace();
            Log.i("HH", "SENDCOMMAND Error " + command + " with error: " + e.toString());
        }
    }

    public void settingsOpenClicked(View view) {
        SettingsPage(true);
    }

    public void startGyroClicked(View view) {
        if(gyro_stream_on){
            sendCommand("DOFF");
            view.setBackground(getResources().getDrawable(R.drawable.arrow_background));
        } else {
            sendCommand("DON");
            initial_x = x;
            initial_y = y;
            initial_z = z;
            view.setBackground(getResources().getDrawable(R.drawable.on_arrow_background));
        }

        gyro_stream_on = !gyro_stream_on;
    }

    public void connectedStatus() {
        final Handler handler = new Handler();
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(true){

                    if(connected){

                        // request values from the robot but only after outputstream is ready
                        /*
                        if(!requested_values){
                            if(mmOutputStream!=null){
                                requested_values = true;
                                sendCommand("V");
                            }
                        }
                        */

                        if(!state_of_connect_button){
                            state_of_connect_button = true;
                            handler.post(new Runnable() {
                                public void run() {
                                    connected_status.setText("Connected");
                                    connected_status.setTextColor(getResources().getColor(R.color.connected));
                                }
                            });
                        }
                    } else {
                        requested_values = false;
                        if(state_of_connect_button){
                            state_of_connect_button = false;
                            handler.post(new Runnable() {
                                public void run() {
                                    connected_status.setText("Not Connected");
                                    connected_status.setTextColor(getResources().getColor(R.color.not_connected));
                                }
                            });
                        }
                    }
                }
            }
        }).start();
    }

    public void connectClicked(View view) {
        connected_status.setText("Connecting");
        requested_values = false;
        connected_status.setTextColor(getResources().getColor(R.color.connecting));
        new Thread(new Runnable() {
            @Override
            public void run() {
                findBT();
            }
        }).start();
    }

    public void sendValuesClicked(View view) {
        // send values to robbotu
        for(int v=0; v<values_to_send.size()-additional_parameters; v++){
            sendCommand("PID " + v + " " + values_to_send.get(v));
        }
        sendCommand("S " + values_to_send.get(values_to_send.size()-2));
        sendCommand("B " + values_to_send.get(values_to_send.size()-1));
    }

    public void updateRangesClicked(View view) {
        int balance_index = mins.size()-1;
        mins.set(balance_index, values_to_send.get(balance_index) - balance_track_delta_on_each_side);
        maxes.set(balance_index, values_to_send.get(balance_index) + balance_track_delta_on_each_side);

        // don't apply zero to the track bar of the balance since it has a value alrdy (balance above)
        if(values_to_send.get(balance_index) > maxes.get(balance_index))
            values_to_send.set(balance_index, maxes.get(balance_index));
        else if(values_to_send.get(balance_index) < mins.get(balance_index))
            values_to_send.set(balance_index, mins.get(balance_index));

        int value_in_percent = percent_from_value(balance_index, values_to_send.get(balance_index));

        track_elements.get(balance_index).setCurrentValue(value_in_percent);


        int switch_index = mins.size()-2;
        float min = values_to_send.get(switch_index) - min_max_deltas;
        mins.set(switch_index, min < 0 ? 0 : min);
        maxes.set(switch_index, values_to_send.get(switch_index) + min_max_deltas);

        // don't apply zero to the track bar of the balance since it has a value alrdy (balance above)
        if(values_to_send.get(switch_index) > maxes.get(switch_index))
            values_to_send.set(switch_index, maxes.get(switch_index));
        else if(values_to_send.get(switch_index) < mins.get(switch_index))
            values_to_send.set(switch_index, mins.get(switch_index));

        value_in_percent = (int) (( values_to_send.get(switch_index) - mins.get(switch_index) ) * 100 / (maxes.get(switch_index)-mins.get(switch_index)));
        if(value_in_percent>100)
            value_in_percent = 100;
        else if(value_in_percent<0)
            value_in_percent = 0;
        track_elements.get(switch_index).setCurrentValue(value_in_percent);
    }

    private int percent_from_value(int index, Float aFloat) {
        int value_in_percent = (int) (( aFloat - mins.get(index) ) * 100 / (maxes.get(index)-mins.get(index)));
        if(value_in_percent>100)
            value_in_percent = 100;
        else if(value_in_percent<0)
            value_in_percent = 0;
        return value_in_percent;
    }

    public void requestValuesClicked(View view) {
        sendCommand("VALUES");
    }
}
