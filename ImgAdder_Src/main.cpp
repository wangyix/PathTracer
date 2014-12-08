#include <stdio.h>
#include <iostream>
#include <algorithm>
#include "lodepng.h"

int main(int argc, const char* argv[]) {

    if (argc < 4) {
        std::cout << "Specify output file followed by 2 or more png files." << std::endl;
        return 0;
    }
    std::string outputFilename = argv[1];

    std::vector<std::string> filenames;
    for (int i = 2; i < argc; i++) {
        filenames.push_back(argv[i]);
    }
    
    // read first img
    std::vector<unsigned char> sumImage;
    unsigned int width, height;
    unsigned int error = lodepng::decode(sumImage, width, height, filenames[0]);
    if (error) {
        std::cout << "Error reading " << filenames[0] << ": " << lodepng_error_text(error) << std::endl;
        return 1;
    }

    // set alpha of pixels to 1
    for (size_t j = 0; j < sumImage.size(); j += 4) {
        sumImage[j + 3] = 255;
    }

    // add the other images to it
    for (size_t i = 1; i < filenames.size(); i++) {
        // read image
        std::vector<unsigned char> image;
        error = lodepng::decode(image, width, height, filenames[i]);
        if (error) {
            std::cout << "Error reading " << filenames[i] << ": " << lodepng_error_text(error) << std::endl;
            return 1;
        }

        // add image to sumImage
        for (size_t j = 0; j < sumImage.size(); j += 4) {
            sumImage[j] = (unsigned char)std::min(sumImage[j] + image[j], 255);
            sumImage[j + 1] = (unsigned char)std::min(sumImage[j + 1] + image[j + 1], 255);
            sumImage[j + 2] = (unsigned char)std::min(sumImage[j + 2] + image[j + 2], 255);
        }
    }

    // write sumimage
    error = lodepng::encode(outputFilename, sumImage, width, height);
    if (error) {
        std::cout << "Error writing " << outputFilename << ": " << lodepng_error_text(error) << std::endl;
        return 1;
    }

    return 0;
}
