package io.github.s5uishida.iot.device.tisensortag.cc2650.driver;

import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;

import org.bluez.Battery1;
import org.bluez.exceptions.BluezFailedException;
import org.bluez.exceptions.BluezInProgressException;
import org.bluez.exceptions.BluezInvalidOffsetException;
import org.bluez.exceptions.BluezInvalidValueLengthException;
import org.bluez.exceptions.BluezNotAuthorizedException;
import org.bluez.exceptions.BluezNotPermittedException;
import org.bluez.exceptions.BluezNotSupportedException;
import org.freedesktop.dbus.exceptions.DBusException;
import org.freedesktop.dbus.handlers.AbstractPropertiesChangedHandler;
import org.freedesktop.dbus.interfaces.Properties.PropertiesChanged;
import org.freedesktop.dbus.types.Variant;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.github.hypfvieh.DbusHelper;
import com.github.hypfvieh.bluetooth.DeviceManager;
import com.github.hypfvieh.bluetooth.DiscoveryFilter;
import com.github.hypfvieh.bluetooth.DiscoveryTransport;
import com.github.hypfvieh.bluetooth.wrapper.BluetoothBattery;
import com.github.hypfvieh.bluetooth.wrapper.BluetoothDevice;
import com.github.hypfvieh.bluetooth.wrapper.BluetoothGattCharacteristic;
import com.github.hypfvieh.bluetooth.wrapper.BluetoothGattDescriptor;
import com.github.hypfvieh.bluetooth.wrapper.BluetoothGattService;

/*
 * Refer to http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide
 *
 * @author s5uishida
 *
 */
public class CC2650Driver {
	private static final Logger LOG = LoggerFactory.getLogger(CC2650Driver.class);

	/*
	 * Constants for using CC2650
	 */
	public static final String NAME									= "CC2650 SensorTag";

	public static final int NOTIFICATION_PERIOD_UNIT_MILLIS		= 10;
	public static final int NOTIFICATION_PERIOD_MAX_MILLIS		= 2550;

	public static final int TEMPERATURE_NOTIFICATION_PERIOD_MIN_MILLIS		= 300;
	public static final int TEMPERATURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS	= 1000;
	public static final int HUMIDITY_NOTIFICATION_PERIOD_MIN_MILLIS			= 100;
	public static final int HUMIDITY_NOTIFICATION_PERIOD_DEFAULT_MILLIS		= 1000;
	public static final int PRESSURE_NOTIFICATION_PERIOD_MIN_MILLIS			= 100;
	public static final int PRESSURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS		= 1000;
	public static final int OPTICAL_NOTIFICATION_PERIOD_MIN_MILLIS			= 100;
	public static final int OPTICAL_NOTIFICATION_PERIOD_DEFAULT_MILLIS		= 800;
	public static final int MOVEMENT_NOTIFICATION_PERIOD_MIN_MILLIS			= 100;
	public static final int MOVEMENT_NOTIFICATION_PERIOD_DEFAULT_MILLIS		= 1000;

	public static final int GYROSCOPE_X_ENABLE						= 0x00000400;
	public static final int GYROSCOPE_Y_ENABLE						= 0x00000200;
	public static final int GYROSCOPE_Z_ENABLE						= 0x00000100;

	public static final int ACCELEROMETER_X_ENABLE					= 0x00002000;
	public static final int ACCELEROMETER_Y_ENABLE					= 0x00001000;
	public static final int ACCELEROMETER_Z_ENABLE					= 0x00000800;

	public static final int MAGNETOMETER_ENABLE						= 0x00004000;

	public static final int WAKE_ON_MOTION_ENABLE					= 0x00008000;

	public static final int ACCELEROMETER_RANGE_2G					= 0x00000000;
	public static final int ACCELEROMETER_RANGE_4G					= 0x00000001;
	public static final int ACCELEROMETER_RANGE_8G					= 0x00000002;
	public static final int ACCELEROMETER_RANGE_16G				= 0x00000003;

	public static final int MOVEMENT_ALL_ENABLE						= GYROSCOPE_X_ENABLE | GYROSCOPE_Y_ENABLE | GYROSCOPE_Z_ENABLE |
			ACCELEROMETER_X_ENABLE | ACCELEROMETER_Y_ENABLE | ACCELEROMETER_Z_ENABLE | MAGNETOMETER_ENABLE;
	public static final int MOVEMENT_ALL_DISABLE					= 0x00000000;

	// Firmware revision
	public static final String UUID_FIRMWARE_REVISION_SERVICE		= "0000180a-0000-1000-8000-00805f9b34fb";
	public static final String UUID_FIRMWARE_REVISION_DATA			= "00002a26-0000-1000-8000-00805f9b34fb";

	// Battery level
	public static final String UUID_BATTERY_LEVEL_SERVICE			= "0000180f-0000-1000-8000-00805f9b34fb";
	public static final String UUID_BATTERY_LEVEL_DATA				= "00002a19-0000-1000-8000-00805f9b34fb";
	public static final float BATTERY_LEVEL_ENABLED_VERSION		= (float)1.3;

	// IR temperature sensor
	public static final String UUID_TEMPERATURE_SERVICE			= "f000aa00-0451-4000-b000-000000000000";
	public static final String UUID_TEMPERATURE_DATA				= "f000aa01-0451-4000-b000-000000000000";
	public static final String UUID_TEMPERATURE_CONFIG				= "f000aa02-0451-4000-b000-000000000000";
	public static final String UUID_TEMPERATURE_PERIOD				= "f000aa03-0451-4000-b000-000000000000";

	// Humidity sensor
	public static final String UUID_HUMIDITY_SERVICE				= "f000aa20-0451-4000-b000-000000000000";
	public static final String UUID_HUMIDITY_DATA					= "f000aa21-0451-4000-b000-000000000000";
	public static final String UUID_HUMIDITY_CONFIG					= "f000aa22-0451-4000-b000-000000000000";
	public static final String UUID_HUMIDITY_PERIOD					= "f000aa23-0451-4000-b000-000000000000";

	// Pressure sensor
	public static final String UUID_PRESSURE_SERVICE				= "f000aa40-0451-4000-b000-000000000000";
	public static final String UUID_PRESSURE_DATA					= "f000aa41-0451-4000-b000-000000000000";
	public static final String UUID_PRESSURE_CONFIG					= "f000aa42-0451-4000-b000-000000000000";
	public static final String UUID_PRESSURE_PERIOD					= "f000aa44-0451-4000-b000-000000000000";

	// Optical sensor
	public static final String UUID_OPTICAL_SERVICE					= "f000aa70-0451-4000-b000-000000000000";
	public static final String UUID_OPTICAL_DATA					= "f000aa71-0451-4000-b000-000000000000";
	public static final String UUID_OPTICAL_CONFIG					= "f000aa72-0451-4000-b000-000000000000";
	public static final String UUID_OPTICAL_PERIOD					= "f000aa73-0451-4000-b000-000000000000";

	// Movement sensor (gyroscope, accelerometer and magnetometer)
	public static final String UUID_MOVEMENT_SERVICE				= "f000aa80-0451-4000-b000-000000000000";
	public static final String UUID_MOVEMENT_DATA					= "f000aa81-0451-4000-b000-000000000000";
	public static final String UUID_MOVEMENT_CONFIG					= "f000aa82-0451-4000-b000-000000000000";
	public static final String UUID_MOVEMENT_PERIOD					= "f000aa83-0451-4000-b000-000000000000";

	private static final byte[] ENABLE = {0x01};
	private static final byte[] DISABLE = {0x00};

	private static final int DEFAULT_WAITING_TIME_TO_USE_GATT_AFTER_CONNECTION_MILLIS = 10000;

	/*
	 * Variables for using CC2650
	 */
	private final BluetoothDevice device;
	private final int waitingTimeToUseGattAfterConnection;
	private final String address;
	private final String name;
	private final String adapterDeviceName;
	private final String logPrefix;
	private final BluetoothBattery battery;

	private boolean ready = false;

	private String firmwareVersion = null;
	private float firmwareVersionFloat = (float)0.0;

	private boolean enableTemperature = false;
	private boolean enableHumidity = false;
	private boolean enablePressure = false;
	private boolean enableOptical = false;
	private boolean enableMovement = false;

	private boolean enableTemperatureNotification = false;
	private boolean enableHumidityNotification = false;
	private boolean enablePressureNotification = false;
	private boolean enableOpticalNotification = false;
	private boolean enableMovementNotification = false;

	private int temperatureNotificationPeriod = TEMPERATURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
	private int humidityNotificationPeriod = HUMIDITY_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
	private int pressureNotificationPeriod = PRESSURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
	private int opticalNotificationPeriod = OPTICAL_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
	private int movementNotificationPeriod = MOVEMENT_NOTIFICATION_PERIOD_DEFAULT_MILLIS;

	private int accelerometerRangeBit = 0x00000000;
	private int accelerometerRange = 2;

	/*
	 * Constant for calculating sensor value of CC2650
	 */
	private static final float SCALE_LSB = 0.03125f;

	/*
	 * Calcuration of sensor value of CC2650
	 */
	private static int signed16Bits(byte[] data, int offset) {
		int byte0 = data[offset] & 0xff;
		int byte1 = (int)data[offset + 1];

		return (byte1 << 8) + byte0;
	}

	private static int unsigned16Bits(byte[] data, int offset) {
		int byte0 = data[offset] & 0xff;
		int byte1 = data[offset + 1] & 0xff;

		return (byte1 << 8) + byte0;
	}

	private static int unsigned24Bits(byte[] data, int offset) {
		int byte0 = data[offset] & 0xff;
		int byte1 = data[offset + 1] & 0xff;
		int byte2 = data[offset + 2] & 0xff;

		return (byte2 << 16) + (byte1 << 8) + byte0;
	}

	public static float[] calculateTemperature(byte[] data) {
		float[] temperatures = new float[2];

		int rawObjTemp = unsigned16Bits(data, 0);
		int rawAmbTemp = unsigned16Bits(data, 2);

		temperatures[0] = (rawObjTemp >> 2) * SCALE_LSB;
		temperatures[1] = (rawAmbTemp >> 2) * SCALE_LSB;

		return temperatures;
	}

	public static float calculateHumidity(byte[] data) {
		int rawHum = unsigned16Bits(data, 2);

		rawHum &= ~0x00000003;

		return (rawHum / 65536f) * 100f;
	}

	public static float calculatePressure(byte[] data) {
		int rawValue = unsigned24Bits(data, 3);

		return (float)(rawValue / 100f);
	}

	public static float calculateOptical(byte[] data) {
		int rawData = unsigned16Bits(data, 0);

		int m = rawData & 0x0fff;
		int e = (rawData & 0xf000) >> 12;

		e = (int)((e == 0) ? 1 : (0x00000002 << (e - 1)));

		return (float)(m * (0.01 * e));
	}

	public static float[] calculateGyroscope(byte[] data) {
		float[] gyroscope = new float[3];

		int x = signed16Bits(data, 0);
		int y = signed16Bits(data, 2);
		int z = signed16Bits(data, 4);

		gyroscope[0] = ((float)x) / (65536f / 500f);
		gyroscope[1] = ((float)y) / (65536f / 500f);
		gyroscope[2] = ((float)z) / (65536f / 500f);

		return gyroscope;
	}

	public static float[] calculateAccelerometer(byte[] data, int accelerometerRange) {
		float[] accelerometer = new float[3];

		int x = signed16Bits(data, 6);
		int y = signed16Bits(data, 8);
		int z = signed16Bits(data, 10);

		accelerometer[0] = ((float)x) / (32768f / accelerometerRange);
		accelerometer[1] = ((float)y) / (32768f / accelerometerRange);
		accelerometer[2] = ((float)z) / (32768f / accelerometerRange);

		return accelerometer;
	}

	public static float[] calculateMagnetometer(byte[] data) {
		float[] magnetometer = new float[3];

		int x = signed16Bits(data, 12);
		int y = signed16Bits(data, 14);
		int z = signed16Bits(data, 16);

		magnetometer[0] = x;
		magnetometer[1] = y;
		magnetometer[2] = z;

		return magnetometer;
	}

	private String hexDump(byte[] data) {
		StringBuffer sb = new StringBuffer();
		for (byte data1 : data) {
			sb.append(String.format("%02x ", data1));
		}
		return sb.toString().trim();
	}

	/*
	 * CC2650 operations
	 */
	public CC2650Driver(BluetoothDevice device) {
		this(device, DEFAULT_WAITING_TIME_TO_USE_GATT_AFTER_CONNECTION_MILLIS);
	}

	public CC2650Driver(BluetoothDevice device, int waitingTimeToUseGattAfterConnection) {
		this.device = Objects.requireNonNull(device);
		this.address = device.getAddress();
		this.name = device.getName();
		this.adapterDeviceName = device.getAdapter().getDeviceName();
		this.logPrefix = "[" + adapterDeviceName + "] " + address + " ";

		if (waitingTimeToUseGattAfterConnection < 0) {
			this.waitingTimeToUseGattAfterConnection = DEFAULT_WAITING_TIME_TO_USE_GATT_AFTER_CONNECTION_MILLIS;
			LOG.info(logPrefix + "{} msec specified for waiting time to use GATT after connection is less than 0, so change to default {} msec.",
					waitingTimeToUseGattAfterConnection,
					DEFAULT_WAITING_TIME_TO_USE_GATT_AFTER_CONNECTION_MILLIS, DEFAULT_WAITING_TIME_TO_USE_GATT_AFTER_CONNECTION_MILLIS);
		} else {
			this.waitingTimeToUseGattAfterConnection = waitingTimeToUseGattAfterConnection;
		}
		LOG.info(logPrefix + "{} msec for waiting time to use GATT after connection is set.",
				this.waitingTimeToUseGattAfterConnection);

		Battery1 rawBattery = DbusHelper.getRemoteObject(device.getDbusConnection(), device.getDbusPath(), Battery1.class);
		battery = new BluetoothBattery(rawBattery, device, device.getDbusPath(), device.getDbusConnection());
	}

	public int getWaitingTimeToAcquireGatt() {
		return waitingTimeToUseGattAfterConnection;
	}

	public BluetoothDevice getBluetoothDevice() {
		return device;
	}

	public String getAddress() {
		return address;
	}

	public String getName() {
		return name;
	}

	public String getAdapterDeviceName() {
		return adapterDeviceName;
	}

	private void reset() {
		enableTemperature = false;
		enableHumidity = false;
		enablePressure = false;
		enableOptical = false;
		enableMovement = false;

		enableTemperatureNotification = false;
		enableHumidityNotification = false;
		enablePressureNotification = false;
		enableOpticalNotification = false;
		enableMovementNotification = false;

		temperatureNotificationPeriod = TEMPERATURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
		humidityNotificationPeriod = HUMIDITY_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
		pressureNotificationPeriod = PRESSURE_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
		opticalNotificationPeriod = OPTICAL_NOTIFICATION_PERIOD_DEFAULT_MILLIS;
		movementNotificationPeriod = MOVEMENT_NOTIFICATION_PERIOD_DEFAULT_MILLIS;

		accelerometerRangeBit = 0x00000000;
		accelerometerRange = 2;

		ready = false;
	}

	public boolean connect() throws IOException {
		if (device.isConnected()) {
			LOG.info(logPrefix + "already connected.");
		} else {
			reset();
			LOG.info(logPrefix + "connecting...");

			if (!device.connect()) {
				LOG.warn(logPrefix + "failed to connect");
				return false;
			}

			LOG.info(logPrefix + "connected.");
		}

		try {
			Thread.sleep(waitingTimeToUseGattAfterConnection);
		} catch (InterruptedException e) {
		}

		device.refreshGattServices();

		for (BluetoothGattService service : device.getGattServices()) {
			if (LOG.isDebugEnabled()) {
				LOG.debug(logPrefix + "service - {}", service.getUuid());
			}
			service.refreshGattCharacteristics();
			for (BluetoothGattCharacteristic characteristic : service.getGattCharacteristics()) {
				if (LOG.isDebugEnabled()) {
					LOG.debug(logPrefix + "characteristic - {}", characteristic.getUuid());
				}
			}
		}

		ready = true;

		return true;
	}

	public boolean disconnect() {
		if (device.isConnected()) {
			LOG.info(logPrefix + "disconnecting...");
			boolean ret = device.disconnect();
			if (ret) {
				reset();
				LOG.info(logPrefix + "disconnected.");
			} else {
				LOG.warn(logPrefix + "failed to disconnect.");
			}
			return ret;
		} else {
			reset();
			LOG.info(logPrefix + "already disconnected.");
			return true;
		}
	}

	public boolean isConnected() {
		return device.isConnected();
	}

	public boolean isReady() {
		return ready;
	}

	private BluetoothGattCharacteristic getGattCharacteristic(String serviceUuid, String characteristicUuid) {
		BluetoothGattService service = device.getGattServiceByUuid(serviceUuid);
		if (service == null) {
			throw new UnsupportedOperationException(logPrefix + "serviceUuid:" + serviceUuid + " not found.");
		}

		BluetoothGattCharacteristic characteristic = service.getGattCharacteristicByUuid(characteristicUuid);
		if (characteristic == null) {
			throw new UnsupportedOperationException(logPrefix + "characteristicUuid:" + characteristicUuid + " not found.");
		}

		return characteristic;
	}

	private void configSensor(String serviceUuid, String characteristicUuid, byte[] config) throws IOException {
		try {
			getGattCharacteristic(serviceUuid, characteristicUuid).writeValue(config, null);
		} catch (BluezFailedException | BluezInProgressException | BluezNotPermittedException
				| BluezNotAuthorizedException | BluezNotSupportedException
				| BluezInvalidValueLengthException e) {
			throw new IOException(logPrefix + "serviceUuid:" +  serviceUuid + " characteristicUuid:" + characteristicUuid +
					" config:" + hexDump(config), e);
		}
	}

	private byte[] readSensor(String serviceUuid, String characteristicUuid) throws IOException {
		try {
			return getGattCharacteristic(serviceUuid, characteristicUuid).readValue(null);
		} catch (BluezFailedException | BluezInProgressException | BluezNotPermittedException
				| BluezNotAuthorizedException | BluezNotSupportedException | BluezInvalidOffsetException e) {
			throw new IOException(logPrefix + "serviceUuid:" +  serviceUuid + " characteristicUuid:" + characteristicUuid, e);
		}
	}

	private int setNotifyPeriod(String serviceUuid, String characteristicUuid, int period, int minPeriod, String tag) throws IOException {
		if (period < minPeriod) {
			period = minPeriod;
			LOG.info(logPrefix + "{} msec specified for {} Notification is shorter than {} msec, so change to {} msec.",
					period, tag, minPeriod, minPeriod);
		} else if (period > NOTIFICATION_PERIOD_MAX_MILLIS) {
			period = NOTIFICATION_PERIOD_MAX_MILLIS;
			LOG.info(logPrefix + "{} msec specified for {} Notification is longer than {} msec, so change to {} msec.",
					period, tag, NOTIFICATION_PERIOD_MAX_MILLIS, NOTIFICATION_PERIOD_MAX_MILLIS);
		}
		period /= NOTIFICATION_PERIOD_UNIT_MILLIS;

		try {
			byte[] data = {(byte)(period & 0x000000ff)};
			getGattCharacteristic(serviceUuid, characteristicUuid).writeValue(data, null);
		} catch (BluezFailedException | BluezInProgressException | BluezNotPermittedException
				| BluezNotAuthorizedException | BluezNotSupportedException | BluezInvalidValueLengthException e) {
			throw new IOException(logPrefix + "serviceUuid:" +  serviceUuid + " characteristicUuid:" + characteristicUuid +
					" period:" + period + " minPeriod:" + minPeriod + " tag:" + tag, e);
		}

		int ret = period * NOTIFICATION_PERIOD_UNIT_MILLIS;

		LOG.info(logPrefix + "{} msec for {} Notification is set.", ret, tag);

		return ret;
	}

	/*
	 * Firmware version
	 */
	public String readFirmwareVersion() throws IOException {
		byte[] ret = readSensor(UUID_FIRMWARE_REVISION_SERVICE, UUID_FIRMWARE_REVISION_DATA);
		return new String(ret, "UTF-8");
	}

	public String getFirmwareVersion() {
		if (firmwareVersion == null || firmwareVersion.isEmpty()) {
			try {
				firmwareVersion = readFirmwareVersion();
				firmwareVersionFloat = Float.valueOf(firmwareVersion.split("\\s+")[0]);
			} catch (IOException e) {
				LOG.warn("caught - {}", e.toString());
			}
		}
		return firmwareVersion;
	}

	public float getFirmwareVersionFloat() {
		if (firmwareVersion == null || firmwareVersion.isEmpty()) {
			getFirmwareVersion();
		}
		return firmwareVersionFloat;
	}

	/*
	 * Battery level
	 */

	public boolean isEnableBatteryLevel() {
		if (getFirmwareVersionFloat() >= BATTERY_LEVEL_ENABLED_VERSION) {
			return true;
		} else {
			return false;
		}
	}

	public int readBatteryLevel() throws IOException {
//		byte[] ret = readSensor(UUID_BATTERY_LEVEL_SERVICE, UUID_BATTERY_LEVEL_DATA);
//		return (int)ret[0];

		try {
			if (isEnableBatteryLevel()) {
				return (int)battery.getPercentage();
			} else {
				throw new UnsupportedOperationException(logPrefix + "battery level unsupported.");
			}
		} catch (NullPointerException e) {
			throw new IOException(logPrefix + "battery level not ready yet.", e);
		}
	}

	/*
	 * Temperature
	 */
	public void enableTemperature() throws IOException {
		configSensor(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_CONFIG, ENABLE);
		enableTemperature = true;
		LOG.info(logPrefix + "enable Temperature.");
	}

	public void disableTemperature() throws IOException {
		configSensor(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_CONFIG, DISABLE);
		enableTemperature = false;
		LOG.info(logPrefix + "disable Temperature.");
	}

	public boolean isEnableTemperature() {
		return enableTemperature;
	}

	public float[] readTemperature() throws IOException {
		byte[] ret = readSensor(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_DATA);
		return calculateTemperature(ret);
	}

	public void enableTemperatureNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_DATA).startNotify();
			enableTemperatureNotification = true;
			LOG.info(logPrefix + "start notify of Temperature.");
		} catch (BluezFailedException | BluezInProgressException | BluezNotSupportedException
				| BluezNotPermittedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void disableTemperatureNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_DATA).stopNotify();
			enableTemperatureNotification = false;
			LOG.info(logPrefix + "stop notify of Temperature.");
		} catch (BluezFailedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void setTemperatureNotificationPeriod(int period) throws IOException {
		temperatureNotificationPeriod = setNotifyPeriod(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_PERIOD,
				period, TEMPERATURE_NOTIFICATION_PERIOD_MIN_MILLIS, "Temperature");
	}

	public boolean isNotifyingTemperature() {
//		return getGattCharacteristic(UUID_TEMPERATURE_SERVICE, UUID_TEMPERATURE_DATA).isNotifying();
		return enableTemperatureNotification;
	}

	public int getTemperatureNotificationPeriod() {
		return temperatureNotificationPeriod;
	}

	/*
	 * Humidity
	 */
	public void enableHumidity() throws IOException {
		configSensor(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_CONFIG, ENABLE);
		enableHumidity = true;
		LOG.info(logPrefix + "enable Humidity.");
	}

	public void disableHumidity() throws IOException {
		configSensor(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_CONFIG, DISABLE);
		enableHumidity = false;
		LOG.info(logPrefix + "disable Humidity.");
	}

	public boolean isEnableHumidity() {
		return enableHumidity;
	}

	public float readHumidity() throws IOException {
		byte[] ret = readSensor(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_DATA);
		return calculateHumidity(ret);
	}

	public void enableHumidityNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_DATA).startNotify();
			enableHumidityNotification = true;
			LOG.info(logPrefix + "start notify of Humidity.");
		} catch (BluezFailedException | BluezInProgressException | BluezNotSupportedException
				| BluezNotPermittedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void disableHumidityNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_DATA).stopNotify();
			enableHumidityNotification = false;
			LOG.info(logPrefix + "stop notify of Humidity.");
		} catch (BluezFailedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void setHumidityNotificationPeriod(int period) throws IOException {
		humidityNotificationPeriod = setNotifyPeriod(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_PERIOD,
				period, HUMIDITY_NOTIFICATION_PERIOD_MIN_MILLIS, "Humidity");
	}

	public boolean isNotifyingHumidity() {
//		return getGattCharacteristic(UUID_HUMIDITY_SERVICE, UUID_HUMIDITY_DATA).isNotifying();
		return enableHumidityNotification;
	}

	public int getHumidityNotificationPeriod() {
		return humidityNotificationPeriod;
	}

	/*
	 * Pressure
	 */
	public void enablePressure() throws IOException {
		configSensor(UUID_PRESSURE_SERVICE, UUID_PRESSURE_CONFIG, ENABLE);
		enablePressure = true;
		LOG.info(logPrefix + "enable Pressure.");
	}

	public void disablePressure() throws IOException {
		configSensor(UUID_PRESSURE_SERVICE, UUID_PRESSURE_CONFIG, DISABLE);
		enablePressure = false;
		LOG.info(logPrefix + "disable Pressure.");
	}

	public boolean isEnablePressure() {
		return enablePressure;
	}

	public float readPressure() throws IOException {
		byte[] ret = readSensor(UUID_PRESSURE_SERVICE, UUID_PRESSURE_DATA);
		return calculatePressure(ret);
	}

	public void enablePressureNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_PRESSURE_SERVICE, UUID_PRESSURE_DATA).startNotify();
			enablePressureNotification = true;
			LOG.info(logPrefix + "start notify of Pressure.");
		} catch (BluezFailedException | BluezInProgressException | BluezNotSupportedException
				| BluezNotPermittedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void disablePressureNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_PRESSURE_SERVICE, UUID_PRESSURE_DATA).stopNotify();
			enablePressureNotification = false;
			LOG.info(logPrefix + "stop notify of Pressure.");
		} catch (BluezFailedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void setPressureNotificationPeriod(int period) throws IOException {
		pressureNotificationPeriod = setNotifyPeriod(UUID_PRESSURE_SERVICE, UUID_PRESSURE_PERIOD,
				period, PRESSURE_NOTIFICATION_PERIOD_MIN_MILLIS, "Pressure");
	}

	public boolean isNotifyingPressure() {
//		return getGattCharacteristic(UUID_PRESSURE_SERVICE, UUID_PRESSURE_DATA).isNotifying();
		return enablePressureNotification;
	}

	public int getPressureNotificationPeriod() {
		return pressureNotificationPeriod;
	}

	/*
	 * Optical
	 */
	public void enableOptical() throws IOException {
		configSensor(UUID_OPTICAL_SERVICE, UUID_OPTICAL_CONFIG, ENABLE);
		enableOptical = true;
		LOG.info(logPrefix + "enable Optical.");
	}

	public void disableOptical() throws IOException {
		configSensor(UUID_OPTICAL_SERVICE, UUID_OPTICAL_CONFIG, DISABLE);
		enableOptical = false;
		LOG.info(logPrefix + "disable Optical.");
	}

	public boolean isEnableOptical() {
		return enableOptical;
	}

	public float readOptical() throws IOException {
		byte[] ret = readSensor(UUID_OPTICAL_SERVICE, UUID_OPTICAL_DATA);
		return calculateOptical(ret);
	}

	public void enableOpticalNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_OPTICAL_SERVICE, UUID_OPTICAL_DATA).startNotify();
			enableOpticalNotification = true;
			LOG.info(logPrefix + "start notify of Optical.");
		} catch (BluezFailedException | BluezInProgressException | BluezNotSupportedException
				| BluezNotPermittedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void disableOpticalNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_OPTICAL_SERVICE, UUID_OPTICAL_DATA).stopNotify();
			enableOpticalNotification = false;
			LOG.info(logPrefix + "stop notify of Optical.");
		} catch (BluezFailedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void setOpticalNotificationPeriod(int period) throws IOException {
		opticalNotificationPeriod = setNotifyPeriod(UUID_OPTICAL_SERVICE, UUID_OPTICAL_PERIOD,
				period, OPTICAL_NOTIFICATION_PERIOD_MIN_MILLIS, "Optical");
	}

	public boolean isNotifyingOptical() {
//		return getGattCharacteristic(UUID_OPTICAL_SERVICE, UUID_OPTICAL_DATA).isNotifying();
		return enableOpticalNotification;
	}

	public int getOpticalNotificationPeriod() {
		return opticalNotificationPeriod;
	}

	/*
	 * Movement
	 */
	/*
	 * Set up each sensor related to movement.
	 *
	 * @param config specify various settings with bit masks.
	 * 					GYROSCOPE_X_ENABLE
	 * 					GYROSCOPE_Y_ENABLE
	 * 					GYROSCOPE_Z_ENABLE
	 * 					ACCELEROMETER_X_ENABLE
	 * 					ACCELEROMETER_Y_ENABLE
	 * 					ACCELEROMETER_Z_ENABLE
	 * 					MAGNETOMETER_ENABLE
	 * 					WAKE_ON_MOTION_ENABLE
	 * 					----------------------
	 * 					MOVEMENT_ALL_ENABLE = all enable except WAKE_ON_MOTION_ENABLE
	 * 					MOVEMENT_ALL_DISABLE = all disable
	 * @throws IOException on error
	 */
	public void enableMovement(int config) throws IOException {
		accelerometerRangeBit = ACCELEROMETER_RANGE_16G & config;
		accelerometerRange = (int)Math.pow(2, accelerometerRangeBit + 1);

		byte[] data = {(byte)(0x000000ff & (config >> 8)), (byte)(0x000000ff & config)};
		configSensor(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_CONFIG, data);

		if ((config & MOVEMENT_ALL_ENABLE) > 0) {
			enableMovement = true;
		} else {
			enableMovement = false;
		}
		LOG.info(logPrefix + "config Movement[{}].", String.format("%04x", config));
	}

	public void disableMovement() throws IOException {
		byte[] data = {(byte)(0x000000ff & (MOVEMENT_ALL_DISABLE >> 8)), (byte)(0x000000ff & MOVEMENT_ALL_DISABLE)};
		configSensor(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_CONFIG, data);
		enableMovement = false;
		LOG.info(logPrefix + "disable Movement.");
	}

	public boolean isEnableMovement() {
		return enableMovement;
	}

	public float[] readMovement() throws IOException {
		byte[] ret = readSensor(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_DATA);
		float[] gyr = calculateGyroscope(ret);
		float[] acc = calculateAccelerometer(ret, accelerometerRange);
		float[] mag = calculateMagnetometer(ret);
		float[] allRet = {gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]};
		return allRet;
	}

	public void enableMovementNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_DATA).startNotify();
			enableMovementNotification = true;
			LOG.info(logPrefix + "start notify of Movement.");
		} catch (BluezFailedException | BluezInProgressException | BluezNotSupportedException
				| BluezNotPermittedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void disableMovementNotification() throws IOException {
		try {
			getGattCharacteristic(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_DATA).stopNotify();
			enableMovementNotification = false;
			LOG.info(logPrefix + "stop notify of Movement.");
		} catch (BluezFailedException e) {
			throw new IOException(logPrefix, e);
		}
	}

	public void setMovementNotificationPeriod(int period) throws IOException {
		movementNotificationPeriod = setNotifyPeriod(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_PERIOD,
				period, MOVEMENT_NOTIFICATION_PERIOD_MIN_MILLIS, "Movement");
	}

	public boolean isNotifyingMovement() {
//		return getGattCharacteristic(UUID_MOVEMENT_SERVICE, UUID_MOVEMENT_DATA).isNotifying();
		return enableMovementNotification;
	}

	public int getMovementNotificationPeriod() {
		return movementNotificationPeriod;
	}

	public int getAccelerometerRange() {
		return accelerometerRange;
	}

	/******************************************************************************************************************
	 * Sample main
	 ******************************************************************************************************************/
	public static void main(String[] args) throws IOException, InterruptedException, DBusException {
		testRead();
	}

	public static void testRead() throws IOException, InterruptedException, DBusException {
		DeviceManager manager = null;
		try {
			manager = DeviceManager.getInstance();
		} catch (IllegalStateException e) {
			LOG.info("caught - {}", e.toString());
			try {
				manager = DeviceManager.createInstance(false);
			} catch (DBusException e1) {
				throw new IllegalStateException(e1);
			}
		}

		Map<DiscoveryFilter, Object> filter = new HashMap<DiscoveryFilter, Object>();
		filter.put(DiscoveryFilter.Transport, DiscoveryTransport.LE);
		manager.setScanFilter(filter);

		BluetoothDevice selectedDevice = null;
		for (BluetoothDevice device : manager.scanForBluetoothDevices("hci0", 1000)) {
			String deviceName = device.getName();
			LOG.info("[{}] {} {}", device.getAdapter().getDeviceName(), device.getAddress(), deviceName);
			if (deviceName.equals(CC2650Driver.NAME)) {
				selectedDevice = device;
				break;
			}
		}
		if (selectedDevice == null) {
			throw new IllegalStateException("device not found.");
		}

		CC2650Driver cc2650 = new CC2650Driver(selectedDevice);

		String logPrefix = "[" + cc2650.getAdapterDeviceName() + "] " + cc2650.getAddress() + " ";

		cc2650.connect();
		LOG.info(logPrefix + "firmware:{}", cc2650.getFirmwareVersion());
		LOG.info(logPrefix + "battery:{}", cc2650.readBatteryLevel());

		cc2650.enableTemperature();
		cc2650.enableHumidity();
		cc2650.enablePressure();
		cc2650.enableOptical();
		cc2650.enableMovement(CC2650Driver.MOVEMENT_ALL_ENABLE | CC2650Driver.ACCELEROMETER_RANGE_16G);

		for (int i = 0; i < 5; i++) {
			float[] temp = cc2650.readTemperature();
			LOG.info(logPrefix + "obj:{} amb:{}", temp[0], temp[1]);

			float humidity = cc2650.readHumidity();
			LOG.info(logPrefix + "humidity:{}", humidity);

			float pressure = cc2650.readPressure();
			LOG.info(logPrefix + "pressure:{}", pressure);

			float optical = cc2650.readOptical();
			LOG.info(logPrefix + "optical:{}", optical);

			float[] mov = cc2650.readMovement();
			LOG.info(logPrefix + "gyr[x]:{}", mov[0]);
			LOG.info(logPrefix + "gyr[y]:{}", mov[1]);
			LOG.info(logPrefix + "gyr[z]:{}", mov[2]);
			LOG.info(logPrefix + "acc[x]:{}", mov[3]);
			LOG.info(logPrefix + "acc[y]:{}", mov[4]);
			LOG.info(logPrefix + "acc[z]:{}", mov[5]);
			LOG.info(logPrefix + "mag[x]:{}", mov[6]);
			LOG.info(logPrefix + "mag[y]:{}", mov[7]);
			LOG.info(logPrefix + "mag[z]:{}", mov[8]);

			Thread.sleep(1000);
		}

		cc2650.disconnect();
	}

	public static void testNotification() throws IOException, InterruptedException, DBusException {
		DeviceManager manager = null;
		try {
			manager = DeviceManager.getInstance();
		} catch (IllegalStateException e) {
			LOG.info("caught - {}", e.toString());
			try {
				manager = DeviceManager.createInstance(false);
			} catch (DBusException e1) {
				throw new IllegalStateException(e1);
			}
		}

		Map<DiscoveryFilter, Object> filter = new HashMap<DiscoveryFilter, Object>();
		filter.put(DiscoveryFilter.Transport, DiscoveryTransport.LE);
		manager.setScanFilter(filter);

		BluetoothDevice selectedDevice = null;
		for (BluetoothDevice device : manager.scanForBluetoothDevices("hci0", 1000)) {
			String deviceName = device.getName();
			LOG.info("[{}] {} {}", device.getAdapter().getDeviceName(), device.getAddress(), deviceName);
			if (deviceName.equals(CC2650Driver.NAME)) {
				selectedDevice = device;
				break;
			}
		}
		if (selectedDevice == null) {
			throw new IllegalStateException("device not found.");
		}

		CC2650Driver cc2650 = new CC2650Driver(selectedDevice);

		String logPrefix = "[" + cc2650.getAdapterDeviceName() + "] " + cc2650.getAddress() + " ";

		Map<String, CC2650Driver> deviceMap = new HashMap<String, CC2650Driver>();
		deviceMap.put(selectedDevice.getAddress(), cc2650);
		manager.registerPropertyHandler(new MyPropertiesChangedHandler(deviceMap));

		cc2650.connect();
		LOG.info(logPrefix + "firmware:{}", cc2650.getFirmwareVersion());
		LOG.info(logPrefix + "battery:{}", cc2650.readBatteryLevel());

		cc2650.enableTemperature();
		cc2650.setTemperatureNotificationPeriod(1000);
		cc2650.enableTemperatureNotification();
		LOG.info(logPrefix + "isNotifyingTemperature:{}", cc2650.isNotifyingTemperature());

		cc2650.enableHumidity();
		cc2650.setHumidityNotificationPeriod(1000);
		cc2650.enableHumidityNotification();
		LOG.info(logPrefix + "isNotifyingHumidity:{}", cc2650.isNotifyingHumidity());

		cc2650.enablePressure();
		cc2650.setPressureNotificationPeriod(1000);
		cc2650.enablePressureNotification();
		LOG.info(logPrefix + "isNotifyingPressure:{}", cc2650.isNotifyingPressure());

		cc2650.enableOptical();
		cc2650.setOpticalNotificationPeriod(1000);
		cc2650.enableOpticalNotification();
		LOG.info(logPrefix + "isNotifyingOptical:{}", cc2650.isNotifyingOptical());

		cc2650.enableMovement(CC2650Driver.MOVEMENT_ALL_ENABLE | CC2650Driver.ACCELEROMETER_RANGE_16G);
		cc2650.setMovementNotificationPeriod(1000);
		cc2650.enableMovementNotification();
		LOG.info(logPrefix + "isNotifyingMovement:{}", cc2650.isNotifyingMovement());

		while (true) {
			Thread.sleep(10000);
		}
	}

	public static void testWakeOnMotion() throws IOException, InterruptedException, DBusException {
		DeviceManager manager = null;
		try {
			manager = DeviceManager.getInstance();
		} catch (IllegalStateException e) {
			LOG.info("caught - {}", e.toString());
			try {
				manager = DeviceManager.createInstance(false);
			} catch (DBusException e1) {
				throw new IllegalStateException(e1);
			}
		}

		Map<DiscoveryFilter, Object> filter = new HashMap<DiscoveryFilter, Object>();
		filter.put(DiscoveryFilter.Transport, DiscoveryTransport.LE);
		manager.setScanFilter(filter);

		BluetoothDevice selectedDevice = null;
		for (BluetoothDevice device : manager.scanForBluetoothDevices("hci0", 1000)) {
			String deviceName = device.getName();
			LOG.info("[{}] {} {}", device.getAdapter().getDeviceName(), device.getAddress(), deviceName);
			if (deviceName.equals(CC2650Driver.NAME)) {
				selectedDevice = device;
				break;
			}
		}
		if (selectedDevice == null) {
			throw new IllegalStateException("device not found.");
		}

		CC2650Driver cc2650 = new CC2650Driver(selectedDevice);

		String logPrefix = "[" + cc2650.getAdapterDeviceName() + "] " + cc2650.getAddress() + " ";

		Map<String, CC2650Driver> deviceMap = new HashMap<String, CC2650Driver>();
		deviceMap.put(selectedDevice.getAddress(), cc2650);
		manager.registerPropertyHandler(new MyPropertiesChangedHandler(deviceMap));

		cc2650.connect();
		LOG.info(logPrefix + "firmware:{}", cc2650.getFirmwareVersion());
		LOG.info(logPrefix + "battery:{}", cc2650.readBatteryLevel());

		cc2650.enableMovement(CC2650Driver.MOVEMENT_ALL_ENABLE | CC2650Driver.WAKE_ON_MOTION_ENABLE | CC2650Driver.ACCELEROMETER_RANGE_16G);
		cc2650.setMovementNotificationPeriod(1000);
		cc2650.enableMovementNotification();
		LOG.info(logPrefix + "isNotifyingMovement:{}", cc2650.isNotifyingMovement());

		while (true) {
			Thread.sleep(10000);
		}
	}
}

/******************************************************************************************************************
 * Sample PropertiesChangedHandler
 ******************************************************************************************************************/
class MyPropertiesChangedHandler extends AbstractPropertiesChangedHandler {
	private static final Logger LOG = LoggerFactory.getLogger(MyPropertiesChangedHandler.class);

	private final Map<String, CC2650Driver> deviceMap;

	public MyPropertiesChangedHandler(Map<String, CC2650Driver> deviceMap) {
		this.deviceMap = deviceMap;
	}

	private Object lookupObject(String dbusPath) {
		Iterator<Entry<String, CC2650Driver>> it = deviceMap.entrySet().iterator();
		while (it.hasNext()) {
			Entry<String, CC2650Driver> entry = it.next();
			BluetoothDevice device = entry.getValue().getBluetoothDevice();
			if  (device.getDbusPath().equals(dbusPath)) {
				return device;
			}
			for (BluetoothGattService service : device.getGattServices()) {
				if (service.getDbusPath().equals(dbusPath)) {
					return service;
				}
				for (BluetoothGattCharacteristic characteristic : service.getGattCharacteristics()) {
					if (characteristic.getDbusPath().equals(dbusPath)) {
						return characteristic;
					}
					for (BluetoothGattDescriptor descriptor : characteristic.getGattDescriptors()) {
						if (descriptor.getDbusPath().equals(dbusPath)) {
							return descriptor;
						}
					}
				}
			}
		}
		return null;
	}

	@Override
	public void handle(PropertiesChanged properties) {
		LOG.debug("path:{} sig:{} interface:{}", properties.getPath(), properties.getName(), properties.getInterfaceName());

		Object btObject = lookupObject(properties.getPath());
		if ((btObject == null) || !(btObject instanceof BluetoothGattCharacteristic)) {
			return;
		}

		BluetoothGattCharacteristic charObject = (BluetoothGattCharacteristic)btObject;
		String uuid = charObject.getUuid();

		Map<String, Variant<?>> propMap = properties.getPropertiesChanged();
		Iterator<Entry<String, Variant<?>>> it = propMap.entrySet().iterator();
		while (it.hasNext()) {
			Entry<String, Variant<?>> entry = it.next();
			Object object = entry.getValue().getValue();

			LOG.debug("[{}] {} -> {} uuid:{}", object.getClass().getName(), entry.getKey(), object, uuid);

			if ((object == null) || !(object instanceof byte[])) {
				continue;
			}

			byte[] data = (byte[])object;
			CC2650Driver cc2650 = deviceMap.get(charObject.getService().getDevice().getAddress());

			String logPrefix = "[" + cc2650.getAdapterDeviceName() + "] " + cc2650.getAddress() + " ";

			if (uuid.equals(CC2650Driver.UUID_TEMPERATURE_DATA)) {
				float[] temp = CC2650Driver.calculateTemperature(data);
				LOG.info(logPrefix + "obj:{} amb:{}", temp[0], temp[1]);
			} else if (uuid.equals(CC2650Driver.UUID_HUMIDITY_DATA)) {
				float humidity = CC2650Driver.calculateHumidity(data);
				LOG.info(logPrefix + "humidity:{}", humidity);
			} else if (uuid.equals(CC2650Driver.UUID_PRESSURE_DATA)) {
				float pressure = CC2650Driver.calculatePressure(data);
				LOG.info(logPrefix + "pressure:{}", pressure);
			} else if (uuid.equals(CC2650Driver.UUID_OPTICAL_DATA)) {
				float optical = CC2650Driver.calculateOptical(data);
				LOG.info(logPrefix + "optical:{}", optical);
			} else if (uuid.equals(CC2650Driver.UUID_MOVEMENT_DATA)) {
				float[] gyr = CC2650Driver.calculateGyroscope(data);
				float[] acc = CC2650Driver.calculateAccelerometer(data, cc2650.getAccelerometerRange());
				float[] mag = CC2650Driver.calculateMagnetometer(data);

				LOG.info(logPrefix + "gyr[x]:{}", gyr[0]);
				LOG.info(logPrefix + "gyr[y]:{}", gyr[1]);
				LOG.info(logPrefix + "gyr[z]:{}", gyr[2]);
				LOG.info(logPrefix + "acc[x]:{}", acc[0]);
				LOG.info(logPrefix + "acc[y]:{}", acc[1]);
				LOG.info(logPrefix + "acc[z]:{}", acc[2]);
				LOG.info(logPrefix + "mag[x]:{}", mag[0]);
				LOG.info(logPrefix + "mag[y]:{}", mag[1]);
				LOG.info(logPrefix + "mag[z]:{}", mag[2]);
			}
		}
	}
}
