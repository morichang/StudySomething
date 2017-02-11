#pragma once

#include "stdafx.h"

class FrameMatching {
private:
	std::string userName;
	std::string date;
	std::string load_path;
	//std::vector<std::string> dind_id;
	
	inline void split(std::vector<std::string> &v, const std::string &input_string, const std::string &delimiter);
	void loadExtractFrameList(std::vector<std::string> &dind_id);
public:
	std::vector<std::string> dind104List;
	std::vector<std::string> dind107List;
	std::vector<std::string> dind108List;
	std::vector<std::string> dind109List;
	std::vector<std::string> dind110List;
	std::vector<std::string> dind111List;
	std::vector<std::string> dind112List;

	FrameMatching();

	void extractFrameMatching(std::vector<std::string> &dind);
};