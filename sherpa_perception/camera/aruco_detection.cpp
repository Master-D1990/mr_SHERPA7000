#include <opencv2/opencv.hpp>
#include "aruco_nano.h" // Das von Ihnen geteilte Header-File

int main() {
    // Kamera öffnen
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Fehler beim Öffnen der Kamera" << std::endl;
        return -1;
    }
    
    // Kameramatrix und Verzerrungskoeffizienten aus npz-Datei laden
    cv::FileStorage fs("calibration_data.xml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["dist_coeffs"] >> distCoeffs;
    fs.release();
    
    // Markergröße in Metern
    float markerSize = 0.05; // 5cm
    
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        
        // Marker erkennen mit ArucoNano
        auto markers = aruconano::MarkerDetector::detect(frame);
        
        for (const auto &m : markers) {
            // Marker zeichnen
            m.draw(frame);
            
            // Marker-Pose schätzen
            auto r_t = m.estimatePose(cameraMatrix, distCoeffs, markerSize);
            cv::Mat rvec = r_t.first;
            cv::Mat tvec = r_t.second;
            
            // Achsen zeichnen
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerSize/2);
            
            // Rotationsmatrix berechnen
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            
            // Z-Winkel (yaw) extrahieren - Vereinfachter Ansatz für RollPitchYaw
            double theta = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0 / CV_PI;
            
            // Distanz berechnen
            double distance = cv::norm(tvec);
            
            // Informationen anzeigen
            std::stringstream ss;
            ss << "ID: " << m.id << " Winkel: " << theta << "° Dist: " << distance << "m";
            cv::putText(frame, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                       0.6, cv::Scalar(0, 255, 0), 2);
        }
        
        // Anzeigen
        cv::imshow("ArucoNano Detection", frame);
        
        // Beenden mit 'q'
        if (cv::waitKey(1) == 'q') break;
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}