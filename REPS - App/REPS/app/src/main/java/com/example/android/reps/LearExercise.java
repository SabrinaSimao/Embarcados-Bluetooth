package com.example.android.reps;

import android.app.Activity;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

/**
 * Created by gabrielaalmeida on 31/05/17.
 */

public class LearExercise extends Activity {
    private final static String TAG = DeviceControlActivity.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";
    private TextView isSerial;
    private TextView mConnectionState;
    private TextView mDataField;
    private TextView counter;
    private String mDeviceName;
    private String mDeviceAddress;
    private BluetoothLeService mBluetoothLeService;
    private boolean mConnected = false;
    private BluetoothGattCharacteristic characteristicRX;
    List<String> accRep4 = new ArrayList<String>();
    HashMap<String, List<List<String>>> exercise_data = new HashMap<String, List<List<String>>>();
    int repeticoes_certas = 0;
    int repeticoes_erradas = 0;
    int primeiro = 1;

    private int rep = 0;



    public final static UUID HM_RX_TX =
            UUID.fromString(SampleGattAttributes.HM_RX_TX);

    private final String LIST_NAME = "NAME";
    private final String LIST_UUID = "UUID";

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            mBluetoothLeService.connect(mDeviceAddress);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                updateConnectionState(R.string.connected);
                invalidateOptionsMenu();
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                updateConnectionState(R.string.disconnected);
                invalidateOptionsMenu();
                clearUI();
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                // Show all the supported services and characteristics on the user interface.
                displayGattServices(mBluetoothLeService.getSupportedGattServices());
            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                displayData(intent.getStringExtra(mBluetoothLeService.EXTRA_DATA));
            }
        }
    };

    private void clearUI() {
        mDataField.setText(R.string.no_data);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.learn_exercise);

        final Intent intent = getIntent();
        mDeviceName = intent.getStringExtra(EXTRAS_DEVICE_NAME);
        mDeviceAddress = intent.getStringExtra(EXTRAS_DEVICE_ADDRESS);

        // Sets up UI references.
        ((TextView) findViewById(R.id.device_address)).setText(mDeviceAddress);
        mConnectionState = (TextView) findViewById(R.id.connection_state);
        // is serial present?
        isSerial = (TextView) findViewById(R.id.isSerial);

        mDataField = (TextView) findViewById(R.id.data_value);

        getActionBar().setTitle(mDeviceName);
        getActionBar().setDisplayHomeAsUpEnabled(true);
        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);
    }

    @Override
    protected void onResume() {
        super.onResume();
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(mDeviceAddress);
            Log.d(TAG, "Connect request result=" + result);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        unregisterReceiver(mGattUpdateReceiver);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.gatt_services, menu);
        if (mConnected) {
            menu.findItem(R.id.menu_connect).setVisible(false);
            menu.findItem(R.id.menu_disconnect).setVisible(true);
        } else {
            menu.findItem(R.id.menu_connect).setVisible(true);
            menu.findItem(R.id.menu_disconnect).setVisible(false);
        }
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch(item.getItemId()) {
            case R.id.menu_connect:
                mBluetoothLeService.connect(mDeviceAddress);
                return true;
            case R.id.menu_disconnect:
                mBluetoothLeService.disconnect();
                return true;
            case android.R.id.home:
                onBackPressed();
                return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private void updateConnectionState(final int resourceId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mConnectionState.setText(resourceId);
            }
        });
    }

    public void buttonOnClickTrue(View view){
        List<String> accBuffer = new ArrayList<String>();
        if(mConnected) {
            mBluetoothLeService.setCharacteristicNotification(characteristicRX,true);
        }
    }

    public void buttonOnClickTrue2(View view){
        if(mConnected) {
            mBluetoothLeService.setCharacteristicNotification(characteristicRX,true);
        }
    }

    public void buttonOnClickFalse(View view){
        if(mConnected) {
            mDataField.setText("");
            mBluetoothLeService.setCharacteristicNotification(characteristicRX,false);
        }
        dataCenter(accBuffer);
    }

    public void buttonOnClickFalse2(View view){
        if(mConnected) {
            mDataField.setText("");
            mBluetoothLeService.setCharacteristicNotification(characteristicRX, false);
        }
        Log.d("Repeticoes Validas: ", String.valueOf(repeticoes_certas));
        Log.d("Repeticoes Invalidas: ", String.valueOf(repeticoes_erradas));
    }

    List<String> accBuffer = new ArrayList<String>();
    private void displayData(String data) {
        if (rep == 0){
            if (data != null) {
                mDataField.setText(data);
                accBuffer.add(data);
            }
        }
        else{
            if (data != null) {
                mDataField.setText(data);
                accRep4.add(data);
                checkIfMov(accRep4);
            }
        }
    }

    private void dataCenter(List<String> accBuffer){
        int count_0 = 0;
        int count_r= 0;
        String zero = "0 \n";
        int anterior_zero = 1;
        String ponto;
        List<String> accRep = new ArrayList<String>();
        List<String> accRep1 = new ArrayList<String>();
        List<String> accRep2 = new ArrayList<String>();
        List<String> accRep3 = new ArrayList<String>();
        List<List<String>> data = new ArrayList<>();



        for (int i =0; i <accBuffer.size(); i++){
            ponto= accBuffer.get(i);
            Log.d("accBuffer:", String.valueOf(accBuffer.get(i)));
            if (ponto.equals(zero)){
                count_0 +=1;
            }
            else{
                count_0 = 0;
                accRep.add(ponto);
                anterior_zero = 1;
            }
            if (count_0 > 6 && anterior_zero !=0){
                accRep.add("0");
                count_0 = 0;
                anterior_zero = 0;
            }

        }
        rep = 1;
        for (int i =0; i <accRep.size(); i++){
            ponto= accRep.get(i);
            Log.d("accRep:", String.valueOf(accRep.get(i)));
            if (ponto.equals("0")){
                count_r +=1;
            }
            else{
                if(count_r<=2){
                    accRep1.add(ponto);
                }
                else{
                    if(count_r<=4){
                        accRep2.add(ponto);
                    }
                    else{
                        if(count_r<=6){
                            accRep3.add(ponto);
                        }
                    }
                }
            }
        }




        for (int i=0; i<accRep1.size();i++){
            Log.d("accRep1:", String.valueOf(accRep1.get(i)));
        }
        Log.d("sizeRep1",String.valueOf(accRep1.size()));

        data.add(accRep1);

        for (int i=0; i<accRep2.size();i++){
            Log.d("accRep2:", String.valueOf(accRep2.get(i)));
        }
        Log.d("sizeRep2",String.valueOf(accRep2.size()));

        data.add(accRep2);

        for (int i=0; i<accRep3.size();i++){
            Log.d("accRep3:", String.valueOf(accRep3.get(i)));
        }
        Log.d("sizeRep3",String.valueOf(accRep3.size()));

        data.add(accRep3);

        exercise_data.put("LegPress", data);

    }

    private void checkIfMov(List<String> accRep4){
        String ponto;
        String zero = "0 \n";
        int anterior_zero = 0;
        int count_0 =0;
        List<String> accRep = new ArrayList<String>();
        int count_r = 0;

        for (int i =0; i <accRep4.size(); i++){
            ponto= accRep4.get(i);
            if (ponto.equals(zero)){
                count_0 +=1;
            }
            else{
                count_0 = 0;
                accRep.add(ponto);
//            Log.d("counter",String.valueOf(count_0));
                anterior_zero = 1;
            }
            if (count_0 > 10 && anterior_zero !=0){
                accRep.add("0");
                count_0 = 0;
                anterior_zero = 0;
            }
        }

        for(int i =0; i <accRep.size(); i++){
            ponto= accRep.get(i);
            Log.d("accRep:", String.valueOf(accRep.get(i)));
            if (ponto.equals("0")){
                count_r +=1;
            }
            else{
                if(primeiro == 1){
                    if(count_r ==3){
                        processData(accRep);
                        accRep.clear();
                        accRep4.clear();
                    }
                    primeiro = 0;
                }
                else{
                    if(count_r == 2){
                        processData(accRep);
                        accRep.clear();
                        accRep4.clear();
                    }
                }
            }
        }
    }
    private void processData(List<String> accRep4){
        String ponto;
        String zero = "0 \n";
        int count_0 = 0;
        List<List<String>> data = new ArrayList<>();
        List<String> accRep = new ArrayList<String>();
        int anterior_zero = 1
                ;
        for (int i =0; i <accRep4.size(); i++){
            ponto= accRep4.get(i);
            if (ponto.equals(zero)){
                count_0 +=1;
            }
            else{
                count_0 = 0;
                accRep.add(ponto);
                anterior_zero = 1;
            }
            if (count_0 > 10 && anterior_zero !=0){
                count_0 = 0;
                anterior_zero = 0;
            }

        }

        data= exercise_data.get("LegPress");
        int data1_size = data.get(0).size();
        int data2_size = data.get(1).size();
        int data3_size = data.get(2).size();
        int menor = accRep.size();
        int menor_p;
        int maior_p;
        int ponto1;
        int ponto2;
        int ponto3;
        int ponto_a;
        double extremo_max;
        double extremo_min;
        int pontos_invalidos=0;
        int valido = 1;

        Log.d("len ponto", String.valueOf(menor));
        Log.d("len ponto1", String.valueOf(data1_size));
        Log.d("len ponto2", String.valueOf(data2_size));
        Log.d("len ponto3", String.valueOf(data3_size));

        if(menor > data1_size || menor > data2_size || menor > data3_size){
            menor = return_menor(data1_size, data2_size, data3_size);
        }
        Log.d("MENOR", String.valueOf(menor));

        for (int i = 0; i<menor; i++){

            ponto1 = str_to_int(data.get(0).get(i));
            ponto2 = str_to_int(data.get(1).get(i));
            ponto3 = str_to_int(data.get(2).get(i));
            ponto_a = str_to_int(accRep.get(i));

            maior_p = return_maior(ponto1, ponto2, ponto3);
            menor_p = return_menor(ponto1, ponto2, ponto3);
            Log.d("CALCULAR EXTREMOS","");

            extremo_max = maior_p + maior_p * 0.35;
            extremo_min = menor_p - maior_p * 0.35;

            Log.d("extremo max", String.valueOf(extremo_max));
            Log.d("extremo min", String.valueOf(extremo_min));
            Log.d("ponto_a", String.valueOf(ponto_a));

            if(ponto_a > extremo_max || ponto_a < extremo_min){
                pontos_invalidos+=1;
            }
            if(pontos_invalidos > 3){
                valido = 0;
            }
        }
        Log.d("validade: ", String.valueOf(valido));
        if(valido==1){
            repeticoes_certas+=1;

        }
        else{
            repeticoes_erradas+=1;
        }
        counter.setText(repeticoes_certas);

    }

    public int return_maior(int a, int b, int c) {
        int maior;
        if (a > b) {
            maior = a;
        } else {
            maior = b;
        }
        if (maior < c) {
            maior = c;
        }
        return maior;
    }

    public int return_menor(int a, int b, int c) {
        int menor;
        if (a < b) {
            menor = a;
        } else {
            menor = b;
        }
        if (menor > c) {
            menor = c;
        }
        return menor;
    }

    public int str_to_int(String num){
        String[] parts = num.split("");
        StringBuilder number = new StringBuilder();


        for (int i=0; i<parts.length; i++){
            switch(parts[i]){
                case "0":
                    number.append("0");
                    break;
                case "1":
                    number.append("1");
                    break;
                case "2":
                    number.append("2");
                    break;
                case "3":
                    number.append("3");
                    break;
                case "4":
                    number.append("4");
                    break;
                case "5":
                    number.append("5");
                    break;
                case "6":
                    number.append("6");
                    break;
                case "7":
                    number.append("7");
                    break;
                case "8":
                    number.append("8");
                    break;
                case "9":
                    number.append("9");
                    break;
            }
        }
        String N = number.toString();
        int numero = Integer.parseInt(N);
        return numero;
    }


    // Demonstrates how to iterate through the supported GATT Services/Characteristics.
    // In this sample, we populate the data structure that is bound to the ExpandableListView
    // on the UI.
    private void displayGattServices(List<BluetoothGattService> gattServices) {
        if (gattServices == null) return;
        String uuid = null;
        String unknownServiceString = getResources().getString(R.string.unknown_service);
        ArrayList<HashMap<String, String>> gattServiceData = new ArrayList<HashMap<String, String>>();


        // Loops through available GATT Services.
        for (BluetoothGattService gattService : gattServices) {
            HashMap<String, String> currentServiceData = new HashMap<String, String>();
            uuid = gattService.getUuid().toString();
            currentServiceData.put(
                    LIST_NAME, SampleGattAttributes.lookup(uuid, unknownServiceString));

            // If the service exists for HM 10 Serial, say so.
            currentServiceData.put(LIST_UUID, uuid);
            gattServiceData.add(currentServiceData);

            // get characteristic when UUID matches RX/TX UUID
            characteristicRX = gattService.getCharacteristic(BluetoothLeService.UUID_HM_RX_TX);
        }

    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }

}
