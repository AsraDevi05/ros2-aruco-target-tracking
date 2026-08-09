#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>

int main() {
    // 1. Inisialisasi kamera eksternal
        return -1;
    }

    // Set khusus ke APRILTAG_36h11
    cv::Ptr<cv::aruco::Dictionary> arucoDict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::DetectorParameters> arucoParams =
        cv::aruco::DetectorParameters::create();

    // Range warna kuning dalam format HSV (sesuaikan jika bola kurang peka)
    cv::Scalar lowerYellow(15, 100, 100);
    cv::Scalar upperYellow(35, 255, 255);

    std::cout << "Mendeteksi Robot dan Bola Kuning... Tekan 'q' untuk keluar." << std::endl;

    cv::Mat frame, hsv, mask;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Variabel untuk menampung posisi pusat
        bool hasRobot = false, hasBall = false;
        cv::Point robotCenter, ballCenter;
        int markerId = -1;

        // --- A. DETEKSI ROBOT (AprilTag) ---
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(frame, arucoDict, corners, ids, arucoParams, rejected);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            for (size_t i = 0; i < ids.size(); i++) {
                const auto& c = corners[i];
                int posX = static_cast<int>((c[0].x + c[2].x) / 2);
                int posY = static_cast<int>((c[0].y + c[2].y) / 2);

                robotCenter = cv::Point(posX, posY);
                hasRobot = true;
                markerId = ids[i];

                // Gambar titik merah di robot
                cv::circle(frame, robotCenter, 6, cv::Scalar(0, 0, 255), -1);
                cv::putText(frame, "Robot ID " + std::to_string(markerId),
                            cv::Point(posX - 40, posY - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }
        }

        // --- B. DETEKSI BOLA KUNING (Color Masking) ---
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lowerYellow, upperYellow, mask);

        // Bersihkan noise kecil pada mask warna
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Cari kontur (bentuk blob) dari warna kuning
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Ambil kontur terbesar yang diasumsikan sebagai bola kuning
            auto largestContour = std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(*largestContour);

            // Saring ukuran minimal kontur agar noise kecil tidak ikut terdeteksi
            if (area > 200) {
                cv::Point2f centerF;
                float radius;
                cv::minEnclosingCircle(*largestContour, centerF, radius);
                ballCenter = cv::Point(static_cast<int>(centerF.x), static_cast<int>(centerF.y));
                hasBall = true;

                // Gambar lingkaran hijau membungkus bola kuning
                cv::circle(frame, ballCenter, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, ballCenter, 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "Target Bola", cv::Point(ballCenter.x - 40, ballCenter.y - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }

        // --- C. MENGGAMBAR ARAH PANAH (Robot -> Bola) ---
        if (hasRobot && hasBall) {
            // Gambar garis panah warna biru dari pusat robot ke pusat bola
            cv::arrowedLine(frame, robotCenter, ballCenter, cv::Scalar(255, 0, 0), 3, 8, 0, 0.1);

            // Hitung jarak Euclidean antar titik koordinat
            double dx = ballCenter.x - robotCenter.x;
            double dy = ballCenter.y - robotCenter.y;
            double jarak = std::sqrt(dx * dx + dy * dy);

            // Hitung sudut arah (heading) ke bola dalam derajat
            // Negatif dy karena koordinat Y pixel ke bawah terbalik
            double sudut = std::atan2(-dy, dx) * 180.0 / CV_PI;

            // Tampilkan data di terminal & layar
            std::cout << "Target Terkunci -> Jarak: " << jarak << " px, Sudut Arah: "
                      << sudut << "°" << std::endl;
            cv::putText(frame, "Jarak ke Bola: " + std::to_string(static_cast<int>(jarak)) + " px",
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }

        // Tampilkan jendela gambar
        cv::imshow("Mata Kamera - Panduan Target Robot", frame);
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    return 0;
}
