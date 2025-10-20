#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip> // for hex

// Function to convert hex string to integer
int hexToInt(const std::string& hexStr) {
    int value;
    std::stringstream ss;
    ss << std::hex << hexStr;
    ss >> value;
    return value;
}

// Function to save a frame as a PPM image
void saveFrameAsPPM(const std::vector<std::vector<int>>& frame_data, int frame_number) {
    std::string filename = "frame_" + std::to_string(frame_number) + ".ppm";

    std::ofstream imageFile(filename);

    if (!imageFile) {
        std::cerr << "Error: Could not create PPM file " << filename << std::endl;
        return;
    }

    const int rows = 100;  // Fixed number of rows
    const int cols = 100;  // Fixed number of columns

    // Write PPM header
    imageFile << "P3\n" << cols << " " << rows << "\n255\n";

    // Write pixel data
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int pixel_value = frame_data[i % frame_data.size()][j % frame_data[0].size()];
            imageFile << pixel_value << " " << pixel_value << " " << pixel_value << "  ";
        }
        imageFile << "\n";
    }

    imageFile.close();
}

// Main function to read CSV and generate frames
int main() {
    std::ifstream file("hex_log_20240910_173153.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the CSV file!" << std::endl;
        return 1;
    }

    std::string line;
    std::vector<std::vector<int>> frame_data;
    int frame_number = 0;

    // Skip the first line (header row)
    std::getline(file, line);

    // Process CSV data
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string byte;
        std::vector<int> row_data;

        // Read Elapsed Time and Offset (skip them for now)
        std::getline(ss, byte, ',');  // Elapsed Time
        std::getline(ss, byte, ',');  // Offset

        // Read Byte0 to Byte15
        for (int i = 0; i < 16; i++) {
            std::getline(ss, byte, ',');
            int value = hexToInt(byte); // Convert hex string to integer
            row_data.push_back(value);
        }

        // Add the row to the current frame data
        frame_data.push_back(row_data);

        // Save the current frame as an image every 100
        if (frame_data.size() == 100) {
            saveFrameAsPPM(frame_data, frame_number);
            frame_number++;
            frame_data.clear();  // Clear for the next frame
        }
    }

    return 0;
}
