//class KalmanFilter {
//  public:
//    KalmanFilter(float q = 0.1, float r = 1.0, float initialEstimate = 0.0) {
//      Q = q;         // Bruit de processus
//      R = r;         // Bruit de mesure
//      X = initialEstimate;  // Estimation initiale
//      P = 1.0;       // Erreur d’estimation initiale
//    }
//
//    float update(float measurement) {
//      // Prediction
//      P = P + Q;
//
//      // Kalman Gain
//      float K = P / (P + R);
//
//      // Correction
//      X = X + K * (measurement - X);
//      P = (1 - K) * P;
//
//      return X;
//    }
//
//  private:
//    float Q, R;  // bruit du système et bruit de mesure
//    float X;     // estimation actuelle
//    float P;     // erreur estimée
//};
//
//void kalm_setup (void)
//{
//  KalmanFilter kalman(0.1, 3.0); // Q = bruit de système, R = bruit capteur  
//}
