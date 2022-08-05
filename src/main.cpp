#include <Arduino.h>
#include <ThetisLib.h>
#include <Kalman.h>

// Kalman filter instantiation
#include <Kalman.h>

#define N_state 10 // Number of states - Acceleration (XYZ), Gyroscope (XYZ), Lat, Lon, SOG, COG
#define N_obs 	10 // Number of observation - Acceleration (XYZ), Gyroscope (XYZ), Lat, Lon, SOG, COG
#define r_a 	0.5 // Acceleration measurement noise (m/sec)
#define r_g 	1.0 // Gyroscope measurement noise (deg/sec)
#define r_lat 	1.5 // GPS latitude measurement noise (m - radius of error)
#define r_lon 	1.5 // GPS longitude measurement noise (m - radius of error)
#define r_sog 	1.0 // GPS SOG measurement noise (m/s)
#define r_cog 	1.0 // GPS COG measurement noise (deg)
#define q_a 	0.1 // Process error - acceleration
#define q_g 	0.1 // Process error - gyroscope
#define q_gps 	0.5 // Process error - gps

BLA::Matrix<N_obs> obs; // Observation vector
KALMAN<N_state, N_obs> K; // Kalman filter object
unsigned long currMillis;
float dt;

void setup() {
	Serial.begin(115200);
	while(!Serial); // Wait for serial connection

	// Initialize IMU
    if (!initDSO32()) { // Check IMU initialization
        while(true) blinkCode(IMU_ERROR_CODE); // Block further code execution
    }

	// Initialize GPS
	if (!initGPS()) { // Initialize GPS and check if good
        while(true) blinkCode(GPS_ERROR_CODE); // Block further code execution
    }
    MicroNMEA::sendSentence(GPS, "$PMTK220,1000"); // Set GPS update rate to 1000 ms (1 Hz)

	// Initialize Kalman Filter

	// time evolution matrix (whatever... it will be updated in loop)
    //      ax   ay   az   gx   gy   gz   lat  lon  SOG  COG
    K.F = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

    // measurement matrix
    K.H = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
    
    // measurement covariance matrix
	float r_a2 = r_a*r_a;
	float r_g2 = r_g*r_g;
	float r_lat2 = r_lat*r_lat;
	float r_lon2 = r_lon*r_lon;
	float r_sog2 = r_sog*r_sog;
	float r_cog2 = r_cog*r_cog;
    K.R = { r_a2, 	0.0,    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
            0.0,    r_a2, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
            0.0,    0.0,    r_a2,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,    0.0,	0.0, 	r_g2, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	r_g2, 	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	r_g2,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0, 	r_lat2,	0.0,	0.0,	0.0,
			0.0, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	r_lon2,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	r_sog2,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	r_cog2 };
    
    // model covariance matrix
	float q_a2 = q_a*q_a;
	float q_g2 = q_g*q_g;
	float q_gps2 = q_gps*q_gps;
    K.Q = { q_a2, 	0.0,    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
            0.0,    q_a2, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
            0.0,    0.0,    q_a2,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,    0.0,	0.0, 	q_g2, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	q_g2, 	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	q_g2,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0, 	q_gps2,	0.0,	0.0,	0.0,
			0.0, 	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	q_gps2,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	q_gps2,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	q_gps2 };
    
    currMillis = millis();
}

void loop() {
	// Poll IMU
	dso32.getEvent(&accel, &gyro, &temp);

	// Poll GPS
	while(GPS.available()) { // Check for an available GPS message
        char c = GPS.read();
        // Serial.print(c); // Debug
        nmea.process(c);
    }

	// =========================
    // === RUN KALMAN FILTER ===
    // =========================

	// measurement matrix
    K.H = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nmea.isValid(), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nmea.isValid(), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nmea.isValid(), 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, nmea.isValid() };

    // Use linear acceleration for better accuracy
    BLA::Matrix<N_obs> meas;
    meas(0) = accel.acceleration.x;
    meas(1) = accel.acceleration.y;
    meas(2) = accel.acceleration.z;
	meas(3) = gyro.gyro.x;
	meas(4) = gyro.gyro.y;
	meas(5) = gyro.gyro.z;
	meas(6) = nmea.getLatitude();
	meas(7) = nmea.getLongitude();
	meas(8) = nmea.getSpeed();
	meas(9) = nmea.getCourse();
    obs = meas;
    
    // Update Kalman filter
    K.update(obs);

	// =========================
	// === RUN MAHONY FILTER ===
	// =========================

	

	// Debug Statements
	Serial << "Meas: " << obs << '\r' << '\n';
	Serial << "Est : " << K.x << '\r' << '\n';
	Serial.println();

	delay(1000);
}