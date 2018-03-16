/*
Idée de base :
http://sailboatinstruments.blogspot.fr/2012/06/compass-calibration-on-water.html

Il utilise un "cercle d'or" qui consiste à faire un tour le plus régulièrement
possible en termes de vitesse surface et vitesse de rotation avec barre bloquée.
Il en divise les données recueillies en parts temporelles égales et en déduit une
déviation par rapport à l'angle théorique.

On va utiliser le gyroscope (étalonné pendant 10 secondes) pour avoir une référence
angulaire relativement stable qui pourra être corrigée après traitement en utilisant
le "top" magnétomètre sur un tour complet.
Cela permet de corriger les variations de vitesse (surface et rotation) du cercle d'or.

On a besoin de CurveExpert Basic pour déterminer les coefficients de l'équation en
fonction des données recueillies et du modèle d'équation custom suivant :
a+b*sin(radians(x))+c*cos(radians(x))+d*sin(radians(x)*2)+e*cos(radians(x)*2)
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <elapsedMillis.h>
#include "filtre.h"

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

FPB Froll, Fpitch, Fyaw, Fgyro;

elapsedMillis tick100 = 0;
elapsedMillis tick10 = 0;
int incomingByte = 0;
double totz = 0.0f;
int count = 0;
double zoff = 0.0f;
double gz= 0.0f;
boolean cal = false;
boolean enr = false;
double roll, pitch, yaw = 0.0f;

// Coefficients de l'équation de déviation du compas (CurveExpert)
double A = 0.0;
double B = 0.0;
double C = 0.0;
double D = 0.0;
double E = 0.0;

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -85.952304f, -144.312971f, 76.945700f };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.110872f, -0.034282f, -0.001749f },
                                    { -0.034282f, 1.121473f, 0.006872f },
                                    { -0.001749f, 0.006872f, 1.160688f } };

//float mag_field_strength        = 47.128f;

// Avec Magneto 1.2
// dans son boitier avec RS-422 et câble, bureau

// Offsets X, Y et Z de l'accéléromètre entrés directement dans le FXOS8700
int accel_offset[3]             = { -14, 20, -27 };

void setup(void)
{
  Serial.begin(38400);
  while (!Serial) {
    delay(1);
  }

  Serial.println("c <Entrée> pour calibrer le gyroscope");
  Serial.println("e <Entrée> pour commencer l'enregistrement");

  // GYRO_RANGE_250DPS est trop souvent à 350-355° / tour
  if(!gyro.begin(GYRO_RANGE_500DPS))
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }
  if(!accelmag.begin(ACCEL_RANGE_2G, accel_offset[0], accel_offset[1], accel_offset[2]))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void){
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    switch (incomingByte) {
    case 99:  // 'c'
      Serial.println("Etalonnage du gyroscope. Ne pas bouger !");
      cal = true;
      enr = false;  
      zoff = 0.0f;
      totz = 0.0f;
      break;

    case 101: // 'e'
      if (!cal) {
        Serial.println("Enregistrement commencé. Démarrer le(s) tour(s) !");
        Serial.println(" GYRO \t MAG");
        enr = true;
        totz = yaw * DEG_TO_RAD * 100.0f;
      }
      break;
      
    default:
      break;
    }
  }

  if (tick100 > 10) {
    tick100 -= 10;
    /* Get a new sensor event */
    sensors_event_t gevt;
    sensors_event_t aevt;
    sensors_event_t mevt;
    gyro.getEvent(&gevt);
    accelmag.getEvent(&aevt, &mevt);
  
    // Apply mag offset compensation (base values in uTesla)
    float x = mevt.magnetic.x - mag_offsets[0];
    float y = mevt.magnetic.y - mag_offsets[1];
    float z = mevt.magnetic.z - mag_offsets[2];
  
    // Apply mag soft iron error compensation
    float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

    gz = filtre(Fgyro, gevt.gyro.z, 0.1f); // 19 échantillons - 200ms

    roll = (float) atan2(-aevt.acceleration.y, aevt.acceleration.z);
    pitch = (float) atan2(-aevt.acceleration.x, sqrt(aevt.acceleration.y*aevt.acceleration.y
                          + aevt.acceleration.z*aevt.acceleration.z));
    float norm = sqrt(aevt.acceleration.x * aevt.acceleration.x
                      + aevt.acceleration.y * aevt.acceleration.y
                      + aevt.acceleration.z * aevt.acceleration.z);
    float pitchA = asin(-aevt.acceleration.x / norm);
    float rollA = asin(-aevt.acceleration.y / cos(pitchA) / norm);
    norm = sqrt(mx * mx + my * my + mz * mz);
    mx = mx / norm;
    my = -1 * my / norm;
    mz = mz / norm;
    
    // Tilt-compensation
    float Mx = mx * cos(pitchA) + mz * sin(pitchA);
    float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
    yaw = atan2(-My, Mx);

    roll = filtre(Froll, roll, 0.02f) * RAD_TO_DEG;
    pitch = filtre(Fpitch, pitch, 0.02f) * RAD_TO_DEG;
    if (!isnan(yaw)) yaw = filtre(Fyaw, yaw, 0.013f) * RAD_TO_DEG;
    if (yaw < 0.0f) yaw += 360.0f;
    else if (yaw >= 360.0f) yaw -= 360.0f;
  
    totz -= gz + zoff;
    if (!cal) {
      if (totz < 0.0f) totz += M_PI * 2.0f * 100.0f;
      else if (totz >= M_PI * 2.0f * 100.0f) totz -= M_PI * 2.0f * 100.0f;
    }
    
    if (count == 1000) {
      zoff = totz / 1000;
      totz = 0.0f;
      Serial.print("Etalonnage terminé. zoff = "); Serial.println(zoff, 10);
      cal = false;
      count = 0;
    }
    if (cal) count++;
  }

  if (tick10 > 100) {
    tick10 -= 100;
    if (enr) {
//      Serial.print("ROT:\t"); 
      Serial.print((-gz-zoff) * RAD_TO_DEG);
      Serial.print("\t");
//      Serial.print("totZ:\t");
      Serial.print(totz * RAD_TO_DEG / 100.0f, 3);
//      Serial.print("\t");
//      Serial.print("hdg:");
      Serial.print("\t");
      Serial.print(roll, 3);
      Serial.print("\t");
      Serial.print(pitch, 3);
      Serial.print("\t");
      Serial.println(yaw, 3);
    }
  }
}
