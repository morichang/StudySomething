#include "KinectPointIO.h"

void KinectPointIO::loadBinary(std::string filepath, std::vector<struct float_point>& pointsList){
	pointsList.clear();

	std::ifstream ifs(filepath, std::ios_base::in | std::ios_base::binary);
	if(!ifs) {
		std::cerr << "ファイルオープンに失敗" << std::endl;
	}

	struct float_point data;
	while(!ifs.eof()) {
		ifs.read((char*)&data, sizeof(struct float_point));
		pointsList.push_back(data);
	}
}