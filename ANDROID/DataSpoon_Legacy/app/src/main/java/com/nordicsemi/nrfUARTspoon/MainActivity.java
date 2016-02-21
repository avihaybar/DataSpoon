/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.nordicsemi.nrfUARTspoon;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;


import com.nordicsemi.nrfUARTspoon.UartService;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.res.Configuration;
import android.graphics.Color;
import android.media.Ringtone;
import android.media.RingtoneManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import org.w3c.dom.Text;

public class MainActivity extends Activity implements RadioGroup.OnCheckedChangeListener {
    private static final int REQUEST_SELECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;
    private static final int UART_PROFILE_READY = 10;
    private static final int UART_PROFILE_CONNECTED = 20;
    private static final int UART_PROFILE_DISCONNECTED = 21;
    private static final int STATE_OFF = 10;
    public static final String TAG = "nRFUART";

    TextView mRemoteRssiVal;
    RadioGroup mRg;
    private int mState = UART_PROFILE_DISCONNECTED;
    private UartService mService = null;
    private BluetoothDevice mDevice = null;
    private BluetoothAdapter mBtAdapter = null;
    private ListView messageListView;
    private ArrayAdapter<String> listAdapter;
    private Button btnConnectDisconnect,btnSend,btnStartNewFile;
    private boolean isFileOpen = false;
    private int fileTime = 0;
    private BufferedWriter theWriter = null;
    private EditText edtMessage;
    private TextView fileMessage;
    private SimpleDateFormat formatter;
    private int displayCounter = 0;
	Button [] stateButtons;
    private int buttonState = 5;
    private Button buttonGo;

    private static int stable_counter=0;
    private static double prev_max_val = 0;
    
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        mBtAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBtAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        messageListView = (ListView) findViewById(R.id.listMessage);
        listAdapter = new ArrayAdapter<String>(this, R.layout.message_detail);
        messageListView.setAdapter(listAdapter);
        messageListView.setDivider(null);
        btnConnectDisconnect=(Button) findViewById(R.id.btn_select);
        btnSend=(Button) findViewById(R.id.sendButton);
        btnStartNewFile=(Button) findViewById(R.id.btn_start_new_file);
        edtMessage = (EditText) findViewById(R.id.sendText);
        fileMessage = (TextView) findViewById(R.id.fileStatusText);
    	formatter = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS");
        buttonGo = (Button) findViewById(R.id.btnGO);

        service_init();

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_DISMISS_KEYGUARD);
        
        // state of spoon buttons
        stateButtons = new Button[6];
        stateButtons[0] = (Button)this.findViewById(R.id.state0_button);
		stateButtons[1] = (Button)this.findViewById(R.id.state1_button);
		stateButtons[2] = (Button)this.findViewById(R.id.state2_button);
		stateButtons[3] = (Button)this.findViewById(R.id.state3_button);
		stateButtons[4] = (Button)this.findViewById(R.id.state4_button);
		stateButtons[5] = (Button)this.findViewById(R.id.state5_button);
		stateButtons[5].setEnabled(false);
		buttonState = 5;

        final CheckBox expert = (CheckBox) findViewById(R.id.ckBoxExpertMode);
        final ListView listViewData = (ListView) findViewById(R.id.listMessage);


        buttonGo.setVisibility(View.INVISIBLE);
        listViewData.setVisibility(ListView.INVISIBLE);

        //Expert Mode Option
        findViewById(R.id.ckBoxExpertMode).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (expert.isChecked()) {
                    listViewData.setVisibility(ListView.VISIBLE);
                } else {
                    listViewData.setVisibility(ListView.INVISIBLE);
                }
            }


        });
        // Handler Disconnect & Connect button
        btnConnectDisconnect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //initialize stable sample
                buttonGo.setVisibility(View.INVISIBLE);
                MainActivity.stable_counter=0;

                if (!mBtAdapter.isEnabled()) {
                    Log.i(TAG, "onClick - BT not enabled yet");
                    Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                    startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
                }
                else {
                	if (btnConnectDisconnect.getText().equals("Connect")){
                		
                		//Connect button pressed, open DeviceListActivity class, with popup windows that scan for devices
                		
            			Intent newIntent = new Intent(MainActivity.this, DeviceListActivity.class);
            			startActivityForResult(newIntent, REQUEST_SELECT_DEVICE);
        			} else {
        				//Disconnect button pressed
        				if (mDevice!=null)
        				{
        					mService.disconnect();
        					
        				}
        			}
                }
            }
        });
        // Handler Send button  
        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	EditText editText = (EditText) findViewById(R.id.sendText);
            	String message = editText.getText().toString();
            	byte[] value;
				try {
					//send data to service
					value = message.getBytes("UTF-8");
					mService.writeRXCharacteristic(value);
					//Update the log with time stamp
					String currentDateTimeString = DateFormat.getTimeInstance().format(new Date());
					listAdapter.add("["+currentDateTimeString+"] TX: "+ message);
               	 	messageListView.smoothScrollToPosition(listAdapter.getCount() - 1);
               	 	edtMessage.setText("");
				} catch (UnsupportedEncodingException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
                
            }
        });
        // Handler File button  
        btnStartNewFile.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	// Close an open file
            	if (isFileOpen) {
            		btnStartNewFile.setText("Start new file");
                    fileMessage.setText("File closed");
                    try {
                    	Log.i(TAG, "Closing old file");
                    	isFileOpen = false;
                    	theWriter.flush();
                    	theWriter.close();
                    	theWriter = null;
                    } catch (IOException ioe) {
                    	Log.e(TAG, "Could not close previous file");
                    }
            	} else {
            		// Open a new file            	
            		btnStartNewFile.setText("End file");

            		String currentDateTimeStringFile = formatter.format(new Date());
            		currentDateTimeStringFile += ".txt";
            		File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
            		path.mkdirs();
            		File theFile = new File(path, currentDateTimeStringFile);
            		try {
            			theWriter = new BufferedWriter(new FileWriter(theFile));
            		} catch (IOException ioe) {
            			Log.e(TAG, "Buffered writer not created at" + theFile.getPath());
            			return;

            		}           	
            		fileMessage.setText("Writing to " + theFile.getPath());
            		isFileOpen = true;  
            		fileTime = 0;
            	}
            }
        });
     
        // Set initial UI state
        
    }
    
    public void stateButtonClickHandler(View target) {
		for (int i = 0; i < stateButtons.length; i++) {
			if (target.getId()==stateButtons[i].getId()) {
				stateButtons[i].setEnabled(false);
				buttonState = i;
			} else
				stateButtons[i].setEnabled(true);
		}    	
    }

    
    //UART service connected/disconnected
    private ServiceConnection mServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder rawBinder) {
        		mService = ((UartService.LocalBinder) rawBinder).getService();
        		Log.d(TAG, "onServiceConnected mService= " + mService);
        		if (!mService.initialize()) {
                    Log.e(TAG, "Unable to initialize Bluetooth");
                    finish();
                }

        }

        public void onServiceDisconnected(ComponentName classname) {
       ////     mService.disconnect(mDevice);
        		mService = null;
        }
    };

    private Handler mHandler = new Handler() {
        @Override
        
        //Handler events that received from UART service 
        public void handleMessage(Message msg) {
  
        }
    };

    private final BroadcastReceiver UARTStatusChangeReceiver = new BroadcastReceiver() {

        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            final Intent mIntent = intent;
           //*********************//
            if (action.equals(UartService.ACTION_GATT_CONNECTED)) {
            	 runOnUiThread(new Runnable() {
                     public void run() {
                         	String currentDateTimeString = DateFormat.getTimeInstance().format(new Date());
                             Log.d(TAG, "UART_CONNECT_MSG");
                             btnConnectDisconnect.setText("Disconnect");
                             edtMessage.setEnabled(true);
                             btnSend.setEnabled(true);
                             ((TextView) findViewById(R.id.deviceName)).setText(mDevice.getName()+ " - ready");
                             listAdapter.add("["+currentDateTimeString+"] Connected to: "+ mDevice.getName());
                        	 	messageListView.smoothScrollToPosition(listAdapter.getCount() - 1);
                             mState = UART_PROFILE_CONNECTED;
                     }
            	 });
            }
           
          //*********************//
            if (action.equals(UartService.ACTION_GATT_DISCONNECTED)) {
            	 runOnUiThread(new Runnable() {
                     public void run() {
                    	 	 String currentDateTimeString = DateFormat.getTimeInstance().format(new Date());
                             Log.d(TAG, "UART_DISCONNECT_MSG");
                             btnConnectDisconnect.setText("Connect");
                             edtMessage.setEnabled(false);
                             btnSend.setEnabled(false);
                             ((TextView) findViewById(R.id.deviceName)).setText("Not Connected");
                             listAdapter.add("["+currentDateTimeString+"] Disconnected to: "+ mDevice.getName());
                             mState = UART_PROFILE_DISCONNECTED;
                             mService.close();
                            //setUiState();
                         
                     }
                 });
            }
            
          
          //*********************//
            if (action.equals(UartService.ACTION_GATT_SERVICES_DISCOVERED)) {
             	 mService.enableTXNotification();
            }
          //*********************//
            if (action.equals(UartService.ACTION_DATA_AVAILABLE)) {

                //we sample for a stable data
                double prev_max_val = 0;
                int stable_counter = 0;


                final byte[] txValue = intent.getByteArrayExtra(UartService.EXTRA_DATA);
                 runOnUiThread(new Runnable() {
                     public void run() {
                         String text = "";
                         String textAdapter = "";

                         try {

                             displayCounter++;
                             int millis = ((txValue[2] << 16 | txValue[1] << 8) | txValue[0]) & 0x00FFFFFF;
                             int progmillis = millis;

                             short accelX = (short) ( ( (txValue[4]  & 0xFF) << 8) | (  txValue[3]  & 0xFF) );
                             short accelY = (short) ( ( (txValue[6]  & 0xFF) << 8) | (  txValue[5]  & 0xFF) );
                             short accelZ = (short) ( ( (txValue[8]  & 0xFF) << 8) | (  txValue[7]  & 0xFF) );

                             short gyroX = (short)  ( ( (txValue[10] & 0xFF) << 8) | (  txValue[9]  & 0xFF) );
                             short gyroY = (short)  ( ( (txValue[12] & 0xFF) << 8) | (  txValue[11] & 0xFF) );
                             short gyroZ = (short)  ( ( (txValue[14] & 0xFF) << 8) | (  txValue[13] & 0xFF) );

                             short magnoX = (short) ( ( (txValue[16] & 0x0F) << 8) | (  txValue[15] & 0xFF) );
                             short magnoY = (short) ( ( (txValue[17] & 0xFF) << 4) | ( (txValue[16] & 0X00F0) >> 4 ));
                             short magnoZ = (short) ( ( (txValue[19] & 0xFF) << 8) | (  txValue[18] & 0xFF) );

                             if (((magnoX >> 11) & 0x01) == 1)
                             	magnoX = (short) (0xF000 | magnoX);
                             if (((magnoY >> 11) & 0x01) == 1)
                             	magnoY = (short) (0xF000 | magnoY);

                             // This sets the GREEN 'GO' BUTTON
                             //if (MainActivity.stable_counter == 25 && ((txInt4 < 7) || (txInt4 > -7)) ) {
                             //if (MainActivity.stable_counter == 35 ) {
                             //    buttonGo.setVisibility(View.VISIBLE);
                             //}

                             String newline =  String.format("%07d | %+5d %+5d %+5d | %+5d %+5d %+5d | %+5d %+5d %+5d\n", progmillis, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magnoX, magnoY, magnoZ);

                             /*   String newline =    progmillis + "," +
                                                    accelX + "," + accelY + "," + accelZ + "," +
                                                    gyroX  + "," + gyroY  + "," + gyroZ  + "," +
                                                    magnoX + "," + magnoY + "," + magnoZ;
                               */

                                text += newline + System.getProperty("line.separator");
                                textAdapter = newline; // for the textAdapter, no line separator


                             //    if(((Math.abs(mag_val)-Math.abs(tmpAverage)) < 0.05) && ((averageRoll < 8) && (averageRoll > -8)) ) {
                             //    MainActivity.stable_counter++;
                             //}
                            //else
                             //    MainActivity.stable_counter=0;


                             // update view slowly so as not to overload
            				if (displayCounter % 4 != 0) {
                        		listAdapter.add(textAdapter);
                        	 		messageListView.smoothScrollToPosition(listAdapter.getCount() - 1);
                        	}
            				
                        	// Log to file
            				if (isFileOpen)
            					theWriter.append(text);
                           	 
                         } catch (Exception e) {
                             Log.e(TAG, e.toString());
                         }
                     } // end run
                 }); // end runOnUiThread
             }
           //*********************//
            if (action.equals(UartService.DEVICE_DOES_NOT_SUPPORT_UART)){
            	showMessage("Device doesn't support UART. Disconnecting");
            	mService.disconnect();
            }
            
            
        }
    };

    private void service_init() {
        Intent bindIntent = new Intent(this, UartService.class);
        bindService(bindIntent, mServiceConnection, Context.BIND_AUTO_CREATE);
  
        LocalBroadcastManager.getInstance(this).registerReceiver(UARTStatusChangeReceiver, makeGattUpdateIntentFilter());
    }
    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(UartService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(UartService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(UartService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(UartService.ACTION_DATA_AVAILABLE);
        intentFilter.addAction(UartService.DEVICE_DOES_NOT_SUPPORT_UART);
        return intentFilter;
    }
    @Override
    public void onStart() {
        super.onStart();
    }

    @Override
    public void onDestroy() {
    	 super.onDestroy();
        Log.d(TAG, "onDestroy()");
        
        try {
        	LocalBroadcastManager.getInstance(this).unregisterReceiver(UARTStatusChangeReceiver);
        } catch (Exception ignore) {
            Log.e(TAG, ignore.toString());
        } 
        unbindService(mServiceConnection);
        mService.stopSelf();
        mService= null;
       
    }

    @Override
    protected void onStop() {
        Log.d(TAG, "onStop");
        if (isFileOpen) {
        	try {
        		isFileOpen = false;
        		theWriter.flush();
        		theWriter.close();
        	} catch (IOException ioe){
        		Log.e(TAG, "Could not close file on exit");
        	}
        }
        super.onStop();
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause");
        super.onPause();
    }

    @Override
    protected void onRestart() {
        super.onRestart();
        Log.d(TAG, "onRestart");
    }

    @Override
    public void onResume() {
        super.onResume();
        Log.d(TAG, "onResume");
        if (!mBtAdapter.isEnabled()) {
            Log.i(TAG, "onResume - BT not enabled yet");
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
        }
 
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {

        case REQUEST_SELECT_DEVICE:
        	//When the DeviceListActivity return, with the selected device address
            if (resultCode == Activity.RESULT_OK && data != null) {
                String deviceAddress = data.getStringExtra(BluetoothDevice.EXTRA_DEVICE);
                mDevice = BluetoothAdapter.getDefaultAdapter().getRemoteDevice(deviceAddress);
               
                Log.d(TAG, "... onActivityResultdevice.address==" + mDevice + "mserviceValue" + mService);
                ((TextView) findViewById(R.id.deviceName)).setText(mDevice.getName()+ " - connecting");
                mService.connect(deviceAddress);
                            

            }
            break;
        case REQUEST_ENABLE_BT:
            // When the request to enable Bluetooth returns
            if (resultCode == Activity.RESULT_OK) {
                Toast.makeText(this, "Bluetooth has turned on ", Toast.LENGTH_SHORT).show();

            } else {
                // User did not enable Bluetooth or an error occurred
                Log.d(TAG, "BT not enabled");
                Toast.makeText(this, "Problem in BT Turning ON ", Toast.LENGTH_SHORT).show();
                finish();
            }
            break;
        default:
            Log.e(TAG, "wrong request code");
            break;
        }
    }

    @Override
    public void onCheckedChanged(RadioGroup group, int checkedId) {
       
    }

    
    private void showMessage(String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
  
    }

    @Override
    public void onBackPressed() {
        if (mState == UART_PROFILE_CONNECTED) {
            Intent startMain = new Intent(Intent.ACTION_MAIN);
            startMain.addCategory(Intent.CATEGORY_HOME);
            startMain.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(startMain);
            showMessage("nRFUART's running in background.\n             Disconnect to exit");
        }
        else {
            new AlertDialog.Builder(this)
            .setIcon(android.R.drawable.ic_dialog_alert)
            .setTitle(R.string.popup_title)
            .setMessage(R.string.popup_message)
            .setPositiveButton(R.string.popup_yes, new DialogInterface.OnClickListener()
                {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
   	                finish();
                }
            })
            .setNegativeButton(R.string.popup_no, null)
            .show();
        }
    }
}
