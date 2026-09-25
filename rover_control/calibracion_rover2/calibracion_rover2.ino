/* Calibración guiada CenfoBot rover 2. Arduino C++, ESP32 3.3.11.
 * Abrir esta carpeta completa dentro del repositorio; usa el diagnóstico
 * compartido de ../diagnostico. Serial 115200, comandos con Enter, ! inmediato.
 * Una orden explícita = una prueba. No arranca ninguna secuencia al encender.
 */
#define VRC_CALIBRATION_GUIDE
#define setup diagnosticSetup
#define loop diagnosticLoop
#include "../diagnostico/diagnostico.ino"
#undef setup
#undef loop

bool wheel_map_confirmed = false;
float guided_left = motion_profile.left, guided_right = motion_profile.right;

void guideMenu() {
  Serial.println("ROVER 2 | MENU | CHECK | RESUMEN | ! / STOP");
  Serial.println("1. Ruedas levantadas: M1, M2, REV (15%, 3500 ms; cada comando mueve)");
  Serial.println("2. MAPA OK solo si M1=izquierda, M2=derecha, positivo=adelante");
  Serial.println("3. CERO quieto; esperar GZERO OK. OBSERVAR: girar manualmente izquierda ~90 grados");
  Serial.println("4. SIGNO 1 si giro izquierdo positivo; SIGNO -1 si negativo");
  Serial.println("5. BASE 20 20 | GANANCIAS 0.8 0.08 4 (valores iniciales, ajustar con resultados)");
  Serial.println("6. CERO; esperar OK; RECTO 1000. Luego RECTO 3500");
  Serial.println("7. CERO; esperar OK; GIRO 45, GIRO -45; luego +/-90 y +/-180, una prueba a la vez");
  Serial.println("M1/M2/REV/RECTO/GIRO autorizan UNA prueba sin ARM adicional. No pegar secuencias.");
  Serial.println("CERO vence en 60 s. ! o cualquier comando durante una prueba cancela. Ajustes en RAM.");
}

void guideSummary() {
  Serial.printf("R2 CONFIG mapa=%s signo=%d base_L=%.2f base_R=%.2f kp=%.2f kd=%.2f limite=%.2f\n",
      wheel_map_confirmed ? "CONFIRMADO" : "PENDIENTE", gyro_sign,
      guided_left, guided_right, heading.kp, heading.kd, heading.limit);
  Serial.printf("R2 CAL vigente=%s bias=%.4f; PWM=%s; motors=%s\n",
      gyroCalibrationFresh() ? "SI" : "NO", gyro_bias, pwm_ready ? "OK" : "ERROR", moving ? "ON" : "OFF");
  Serial.println("Resultados: TURN RESULT / HEADING RESULT. RESUMEN no certifica calibracion final.");
}

bool guidedCommand(const String& line) {
  if (line == "MENU") { guideMenu(); return true; }
  if (line == "RESUMEN") { guideSummary(); return true; }
  if (line == "CHECK") {
    stopMotors(); armed = false;
    scanI2c(); initImu(); readLines(10); guideSummary(); return true;
  }
  if (line == "CERO") { startGyroZero(); return true; }
  if (line == "OBSERVAR") { startGyroWatch(10000); return true; }
  if (line == "MAPA OK") {
    armed = false; wheel_map_confirmed = true;
    Serial.println("R2 MAPA confirmado por operador; no es deteccion automatica"); return true;
  }
  if (line == "M1" || line == "M2" || line == "REV") {
    armed = pwm_ready;
    if (line == "M1") runMotor(15, 0, 3500);
    else if (line == "M2") runMotor(0, 15, 3500);
    else runMotor(-15, -15, 3500);
    return true;
  }
  if (line.startsWith("SIGNO ")) {
    int sign; char extra; armed = false;
    if (sscanf(line.c_str(), "SIGNO %d %c", &sign, &extra) == 1 && (sign == 1 || sign == -1)) {
      gyro_sign = sign; Serial.printf("R2 SIGNO=%d confirmado por operador\n", sign);
    } else Serial.println("ERROR: SIGNO 1 o -1 tras verificar giro manual");
    return true;
  }
  if (line.startsWith("BASE ")) {
    float left, right; char extra; armed = false;
    if (sscanf(line.c_str(), "BASE %f %f %c", &left, &right, &extra) == 2 &&
        std::isfinite(left) && std::isfinite(right) && left >= 10 && left <= 25 && right >= 10 && right <= 25) {
      guided_left = left; guided_right = right; guideSummary();
    } else Serial.println("ERROR: BASE <izq 10..25> <der 10..25>");
    return true;
  }
  if (line.startsWith("GANANCIAS ")) {
    float kp, kd, limit; char extra; armed = false;
    if (sscanf(line.c_str(), "GANANCIAS %f %f %f %c", &kp, &kd, &limit, &extra) == 3 &&
        std::isfinite(kp) && std::isfinite(kd) && std::isfinite(limit) &&
        kp >= 0 && kp <= 2 && kd >= 0 && kd <= 0.5F && limit >= 0 && limit <= 5) {
      heading.kp = kp; heading.kd = kd; heading.limit = limit; guideSummary();
    } else Serial.println("ERROR: GANANCIAS kp 0..2 kd 0..0.5 limite 0..5");
    return true;
  }
  if (line.startsWith("RECTO ") || line.startsWith("GIRO ")) {
    armed = false;
    if (!wheel_map_confirmed || !gyroCalibrationFresh() || (gyro_sign != 1 && gyro_sign != -1)) {
      Serial.println("ERROR: verificar MAPA OK, SIGNO y CERO reciente antes de mover con IMU"); return true;
    }
    char extra;
    if (line.startsWith("RECTO ")) {
      int ms;
      if (sscanf(line.c_str(), "RECTO %d %c", &ms, &extra) == 1 && ms >= 1 && ms <= MAX_MOVE_MS) {
        armed = pwm_ready; startStraight(guided_left, guided_right, ms);
      } else Serial.println("ERROR: RECTO ms 1..3500");
    } else {
      float degrees;
      if (sscanf(line.c_str(), "GIRO %f %c", &degrees, &extra) == 1 && std::isfinite(degrees) &&
          fabsf(degrees) >= 10 && fabsf(degrees) <= 180) {
        armed = pwm_ready; startTurn(degrees);
      } else Serial.println("ERROR: GIRO grados +/-10..180");
    }
    return true;
  }
  return false; // Original diagnostic commands remain available.
}

void setup() {
  diagnosticSetup();
  guideMenu();
}

void loop() { diagnosticLoop(); }
